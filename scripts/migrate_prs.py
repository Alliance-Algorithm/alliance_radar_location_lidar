#!/usr/bin/env python3
"""Migrate closed PRs from upstream to alliance repo as closed PRs. Uses curl for API calls."""

import subprocess
import json
import time
import sys
import tempfile
import os
import re

SOURCE_REPO = "HarryPotter1tech/alliance_radar_location_lidar"
TARGET_REPO = "Alliance-Algorithm/alliance_radar_location_lidar"
MAX_BODY_LEN = 60000
DUMMY_BRANCH = "_migrate_placeholder"


def get_token() -> str:
    result = subprocess.run("gh auth token", shell=True, capture_output=True, text=True)
    return result.stdout.strip()


TOKEN = None


def api(method: str, path: str, data: dict | None = None, retries: int = 5) -> dict | list | None:
    global TOKEN
    if not TOKEN:
        TOKEN = get_token()

    for attempt in range(retries):
        cmd = ['curl', '--noproxy', '*', '-s', '-X', method, f'https://api.github.com{path}',
               '-H', f'Authorization: token {TOKEN}',
               '-H', 'Content-Type: application/json']
        if data:
            cmd.extend(['-d', json.dumps(data)])

        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
            if result.returncode == 0 and result.stdout.strip():
                try:
                    return json.loads(result.stdout)
                except json.JSONDecodeError:
                    pass
        except subprocess.TimeoutExpired:
            pass
        except Exception as e:
            print(f"  API error: {e}", file=sys.stderr)

        if attempt < retries - 1:
            time.sleep(5 * (attempt + 1))
    return None


def gh_json(cmd: str, retries: int = 5) -> list | dict:
    for attempt in range(retries):
        env = os.environ.copy()
        env['NO_PROXY'] = '*'
        env['no_proxy'] = '*'
        try:
            result = subprocess.run(f"gh {cmd}", shell=True, capture_output=True, text=True, env=env, timeout=60)
            if result.returncode == 0 and result.stdout.strip():
                return json.loads(result.stdout)
        except subprocess.TimeoutExpired:
            pass
        except Exception as e:
            print(f"  gh error: {e}", file=sys.stderr)

        if attempt < retries - 1:
            time.sleep(5 * (attempt + 1))
    return []


def ensure_dummy_branch():
    global TOKEN
    if not TOKEN:
        TOKEN = get_token()

    # Check if branch exists
    result = api('GET', f'/repos/{TARGET_REPO}/git/refs/heads/{DUMMY_BRANCH}')
    if result:
        return

    # Get default branch
    repo_info = api('GET', f'/repos/{TARGET_REPO}')
    if not repo_info or 'default_branch' not in repo_info:
        print("Cannot access target repo", file=sys.stderr)
        sys.exit(1)
    default_branch = repo_info["default_branch"]

    # Get SHA of default branch
    ref_info = api('GET', f'/repos/{TARGET_REPO}/git/refs/heads/{default_branch}')
    if not ref_info:
        print("Cannot get default branch ref", file=sys.stderr)
        sys.exit(1)
    sha = ref_info["object"]["sha"]

    # Create branch
    result = api('POST', f'/repos/{TARGET_REPO}/git/refs', {
        "ref": f"refs/heads/{DUMMY_BRANCH}",
        "sha": sha
    })
    if result:
        print(f"Created placeholder branch: {DUMMY_BRANCH}")
    else:
        print("Failed to create placeholder branch", file=sys.stderr)


def get_existing_prs() -> set:
    prs = gh_json(f"pr list --repo {TARGET_REPO} --state all --limit 200 --json title")
    return {p["title"] for p in prs}


def create_closed_pr(title: str, body: str) -> int | None:
    global TOKEN
    if not TOKEN:
        TOKEN = get_token()

    with tempfile.NamedTemporaryFile(mode="w", suffix=".md", delete=False) as f:
        f.write(body)
        tmp = f.name
    try:
        # Try gh first
        env = os.environ.copy()
        env['NO_PROXY'] = '*'
        env['no_proxy'] = '*'
        try:
            result = subprocess.run(
                f'gh pr create --repo {TARGET_REPO} --title "{title}" '
                f'--body-file "{tmp}" --head {DUMMY_BRANCH} --base master',
                shell=True, capture_output=True, text=True, timeout=60, env=env
            )
            if result.returncode == 0:
                pr_url = result.stdout.strip()
                match = re.search(r'/pull/(\d+)', pr_url)
                if match:
                    pr_num = int(match.group(1))
                    api('PATCH', f'/repos/{TARGET_REPO}/pulls/{pr_num}', {"state": "closed"})
                    return pr_num
        except subprocess.TimeoutExpired:
            pass

        # Fallback: use REST API
        pr_data = api('POST', f'/repos/{TARGET_REPO}/pulls', {
            "title": title,
            "body": body,
            "head": DUMMY_BRANCH,
            "base": "master"
        })
        if pr_data and "number" in pr_data:
            pr_num = pr_data["number"]
            api('PATCH', f'/repos/{TARGET_REPO}/pulls/{pr_num}', {"state": "closed"})
            return pr_num

        print(f"  Failed to create PR", file=sys.stderr)
        return None
    finally:
        os.unlink(tmp)


def add_comment(pr_num: int, body: str) -> bool:
    global TOKEN
    if not TOKEN:
        TOKEN = get_token()

    result = api('POST', f'/repos/{TARGET_REPO}/issues/{pr_num}/comments', {"body": body})
    return result is not None


def format_pr_body(pr: dict) -> str:
    lines = []
    lines.append("---")
    lines.append(f"*Migrated from https://github.com/{SOURCE_REPO}/pull/{pr['number']}*")
    lines.append(f"*Author: @{pr['author']['login']} | Created: {pr['createdAt']} | Original state: {pr['state']}*")
    lines.append("---")
    lines.append("")

    if pr.get("body"):
        lines.append(pr["body"])

    result = "\n".join(lines)
    if len(result) > MAX_BODY_LEN:
        result = result[:MAX_BODY_LEN] + "\n\n---\n*内容过长，已截断*"
    return result


def format_comment(c: dict) -> str:
    author = c.get("author", {}).get("login", "unknown")
    created = c.get("createdAt", "")
    body = c.get("body", "")
    return f"**@{author}** ({created}):\n\n{body}"


def format_review(r: dict) -> str:
    author = r.get("author", {}).get("login", "unknown")
    state = r.get("state", "")
    body = r.get("body", "")
    if not body:
        return f"**@{author}** approved the pull request." if state == "APPROVED" else f"**@{author}** [{state}]"
    return f"**@{author}** [{state}] ({r.get('createdAt', '')}):\n\n{body}"


def main():
    print(f"Migrating closed PRs from {SOURCE_REPO} to {TARGET_REPO}\n")

    ensure_dummy_branch()

    prs = gh_json(f"pr list --repo {SOURCE_REPO} --state all --limit 100 --json number,title,state")
    if not prs:
        print("No PRs found.")
        return

    print(f"Found {len(prs)} PRs")

    existing = get_existing_prs()
    print(f"Found {len(existing)} existing PRs in target\n")

    success, fail, skip = 0, 0, 0
    for i, pr in enumerate(prs):
        num = pr["number"]
        title = f"[PR #{num}] {pr['title']}"

        if title in existing:
            print(f"[{i+1}/{len(prs)}] Skip (exists): {title}")
            skip += 1
            continue

        print(f"[{i+1}/{len(prs)}] Fetching PR #{num}: {pr['title']}")

        detail = gh_json(
            f"pr view {num} --repo {SOURCE_REPO} "
            f"--json number,title,body,author,createdAt,state,comments,reviews"
        )
        if not detail:
            fail += 1
            continue

        body = format_pr_body(detail)

        new_pr_num = create_closed_pr(title, body)
        if not new_pr_num:
            fail += 1
            continue

        print(f"  Created & closed PR #{new_pr_num}")

        # Add comments
        for c in detail.get("comments", []):
            comment_body = format_comment(c)
            if len(comment_body) > MAX_BODY_LEN:
                comment_body = comment_body[:MAX_BODY_LEN] + "\n\n---\n*内容过长，已截断*"
            add_comment(new_pr_num, comment_body)
            time.sleep(0.5)

        for r in detail.get("reviews", []):
            if r.get("body"):
                review_body = format_review(r)
                if len(review_body) > MAX_BODY_LEN:
                    review_body = review_body[:MAX_BODY_LEN] + "\n\n---\n*内容过长，已截断*"
                add_comment(new_pr_num, review_body)
                time.sleep(0.5)

        success += 1
        time.sleep(3)

    print(f"\nDone: {success} created, {fail} failed, {skip} skipped")


if __name__ == "__main__":
    main()
