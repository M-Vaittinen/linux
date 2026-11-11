#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Count commits with Fixes: tags for files a person maintains/reviews.

Run from the root of a Linux kernel git tree. Settings can be stored in
~/.config/git_kernel_count_beans.ini (or the file given with --config):

    [default]
    identities = Jane Doe, jane@example.com
"""

import argparse
import configparser
import os
import re
import subprocess
import sys
from datetime import date, timedelta

DEFAULT_CONFIG = os.path.expanduser("~/.config/git_kernel_count_beans.ini")


def parse_maintainers(path="MAINTAINERS"):
    """Return list of sections: dict(title, tags={letter: [values]})."""
    sections, cur = [], None
    started = False
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.rstrip("\n")
            if not started:
                started = line == "Maintainers List"
                continue
            if line.startswith(("---", ".. note::")) or line.startswith("          "):
                continue
            if not line.strip():
                cur = None
                continue
            if len(line) > 2 and line[1] == ":" and line[0].isupper():
                if cur is None:
                    continue
                cur["tags"].setdefault(line[0], []).append(line[2:].strip())
            else:
                cur = {"title": line.strip(), "tags": {}}
                sections.append(cur)
    return sections


def matches(entry, idents):
    e = entry.lower()
    return any(i.lower() in e for i in idents)


def pathspecs(tags):
    specs = [":(top)" + p for p in tags.get("F", [])]
    specs += [":(top,exclude)" + p for p in tags.get("X", [])]
    return specs


def fixes_commits(specs, since, until, fixes_only=True):
    cmd = ["git", "log", "--no-merges", "--format=%H", "-E",
           f"--since={since}", f"--until={until} 23:59:59"]
    if fixes_only:
        cmd.append("--grep=^Fixes:")
    cmd += ["--"] + specs
    out = subprocess.run(cmd, capture_output=True, text=True, check=True)
    return set(out.stdout.split())


def dt_binding_files(idents):
    """Return DT binding YAML files listing a matching name in maintainers:."""
    pat = "|".join(re.escape(i) for i in idents)
    out = subprocess.run(["git", "grep", "-l", "-i", "-E", pat, "--",
                          "Documentation/devicetree/bindings/*.yaml"],
                         capture_output=True, text=True)
    files = []
    for path in out.stdout.split():
        in_maint = False
        with open(path, encoding="utf-8", errors="replace") as f:
            for line in f:
                if re.match(r"maintainers:", line):
                    in_maint = True
                    if matches(line, idents):
                        files.append(path)
                        break
                elif in_maint:
                    if re.match(r"\s*(-|#)", line) or not line.strip():
                        if matches(line, idents):
                            files.append(path)
                            break
                    else:
                        in_maint = False
    return files


def authored_files(idents):
    """Return files still in HEAD that the person added or renamed in."""
    esc = "|".join(re.sub(r"([.^$*+?()\[\]{}|\\])", r"\\\1", i)
                   for i in idents)
    out = subprocess.run(
        ["git", "log", "-E", "-i", f"--author={esc}",
         "--diff-filter=AR", "--name-status", "--find-renames", "--format="],
        capture_output=True, text=True, check=True)
    files = {line.split("\t")[-1] for line in out.stdout.splitlines() if line}
    tracked = set(subprocess.run(["git", "ls-files"], capture_output=True,
                                 text=True, check=True).stdout.splitlines())
    return sorted(files & tracked)


def tracked_files(specs):
    out = subprocess.run(["git", "ls-files", "--"] + specs,
                         capture_output=True, text=True, check=True)
    return set(out.stdout.splitlines())


def commit_authors(hashes):
    """Return {hash: 'Name <email>'} for the given commits."""
    out = subprocess.run(["git", "log", "--no-walk=unsorted", "--stdin",
                          "--format=%H\t%an <%ae>"],
                         input="\n".join(hashes), capture_output=True,
                         text=True, check=True)
    return dict(l.split("\t", 1) for l in out.stdout.splitlines() if l)


def store_config(path, idents):
    cp = configparser.ConfigParser()
    cp.read(path)
    if not cp.has_section("default"):
        cp.add_section("default")
    cp.set("default", "identities", ", ".join(idents))
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        cp.write(f)


def load_config(path):
    cp = configparser.ConfigParser()
    cp.read(path)
    raw = cp.get("default", "identities", fallback="")
    return [s.strip() for s in raw.split(",") if s.strip()]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("-p", "--person", action="append", default=[],
                    metavar="NAME_OR_EMAIL",
                    help="name or email as in MAINTAINERS (repeatable; "
                         "overrides config)")
    ap.add_argument("--since", help="start date YYYY-MM-DD "
                    "(default: one year before --until)")
    ap.add_argument("--until", help="end date YYYY-MM-DD (default: today)")
    ap.add_argument("-m", "--maintained", action="store_true",
                    help="files with M: entry (default scope)")
    ap.add_argument("-r", "--reviewed", action="store_true",
                    help="files with R: entry")
    ap.add_argument("-a", "--authored", action="store_true",
                    help="files authored by the person (added/renamed files per git log)")
    ap.add_argument("-d", "--dt-bindings", action="store_true",
                    help="also include DT binding YAML files that list the "
                         "person in maintainers: (often missing from "
                         "MAINTAINERS)")
    ap.add_argument("-s", "--stats", action="store_true",
                    help="append statistics on the number of files found "
                         "per scope (-m, -r, -a, -d)")
    ap.add_argument("--all-commits", action="store_true",
                    help="also count all (non-merge) commits to the matching "
                         "files, not just the Fixes-commits")
    ap.add_argument("--store-as-default", action="store_true",
                    help="store the persons given with -p in the config "
                         "file (created if missing) and use them by default")
    ap.add_argument("-c", "--config", default=DEFAULT_CONFIG,
                    help=f"config file (default: {DEFAULT_CONFIG})")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="list the individual files per scope and the matching commits")
    args = ap.parse_args()

    if args.store_as_default:
        if not args.person:
            ap.error("--store-as-default requires -p/--person")
        store_config(args.config, args.person)
        print(f"Stored default identities in {args.config}", file=sys.stderr)

    idents = args.person or load_config(args.config)
    if not idents:
        ap.error("no person given and none found in " + args.config)

    until = args.until or date.today().isoformat()
    since = args.since or (date.fromisoformat(until) -
                           timedelta(days=365)).isoformat()
    roles = []
    if args.maintained or not (args.reviewed or args.authored):
        roles.append(("M", "maintained"))
    if args.reviewed:
        roles.append(("R", "reviewed"))
    if args.authored:
        roles.append(("A", "authored"))

    if not os.path.exists("MAINTAINERS"):
        ap.error("run from the root of a kernel git tree")

    total, rows = set(), []
    stat_files = {}
    all_total = set()
    for sec in parse_maintainers():
        if not any(k in "MR" for k, _ in roles):
            break
        tags = sec["tags"]
        if not any(matches(v, idents) for k, _ in roles
                   for v in tags.get(k, [])) or "F" not in tags:
            continue
        specs = pathspecs(tags)
        if args.stats or args.verbose:
            for k, _ in roles:
                if any(matches(v, idents) for v in tags.get(k, [])):
                    stat_files.setdefault(k, set()).update(
                        tracked_files(specs))
        found = fixes_commits(specs, since, until)
        if args.all_commits:
            all_total |= fixes_commits(specs, since, until, False)
        rows.append((sec["title"], len(found)))
        total |= found

    if args.authored:
        files = authored_files(idents)
        stat_files["A"] = set(files)
        found = set()
        for i in range(0, len(files), 500):
            chunk = [":(top,literal)" + f for f in files[i:i + 500]]
            found |= fixes_commits(chunk, since, until)
            if args.all_commits:
                all_total |= fixes_commits(chunk, since, until, False)
        rows.append((f"AUTHORED files ({len(files)} files)", len(found)))
        total |= found

    if args.dt_bindings:
        files = dt_binding_files(idents)
        stat_files["D"] = set(files)
        if files:
            specs = [":(top)" + f for f in files]
            found = fixes_commits(specs, since, until)
            if args.all_commits:
                all_total |= fixes_commits(specs, since, until, False)
            rows.append((f"DT BINDINGS from maintainers: ({len(files)} files)",
                         len(found)))
            total |= found

    print(f"Fixes between {since} and {until} for "
          f"{', '.join(idents)} ({'+'.join(n for _, n in roles)} files)")
    for title, n in sorted(rows, key=lambda r: -r[1]):
        print(f"{n:6}  {title}")
    print(f"{len(total):6}  TOTAL (unique Fixes-commits)")
    authors = commit_authors(sorted(total)) if total else {}
    own = {h for h, a in authors.items() if matches(a, idents)}
    print(f"{len(own):6}  of which authored by {', '.join(idents)}")
    if args.verbose:
        names = {"M": "maintained", "R": "reviewed", "A": "authored",
                 "D": "DT bindings"}
        for k in "MRAD":
            if stat_files.get(k):
                print(f"\nFiles ({names[k]}):")
                for f in sorted(stat_files[k]):
                    print(f"  {f}")
    if args.stats:
        names = {"M": "maintained (-m)", "R": "reviewed (-r)",
                 "A": "authored (-a)", "D": "DT bindings (-d)"}
        days = (date.fromisoformat(until) - date.fromisoformat(since)).days + 1
        print(f"\nStatistics: {days} days in time window")
        print("Files found:")
        for k in "MRAD":
            if k in stat_files or k in [r for r, _ in roles] or \
                    (k == "D" and args.dt_bindings):
                print(f"{len(stat_files.get(k, ())):6}  {names[k]}")
        union = set().union(*stat_files.values()) if stat_files else set()
        print(f"{len(union):6}  total unique files")
    all_auth = commit_authors(sorted(all_total)) if all_total else {}
    if args.all_commits:
        pct = 100 * len(total) / len(all_total) if all_total else 0
        print(f"{len(all_total):6}  ALL unique commits to the files "
              f"({pct:.1f}% are Fixes-commits)")
        n = sum(1 for a in all_auth.values() if matches(a, idents))
        print(f"{n:6}  of which authored by {', '.join(idents)}")
    if args.verbose:
        def show(title, hashes, auth):
            print(f"\n{title} (* = authored by the searched person):")
            for h in sorted(hashes):
                subj = subprocess.run(["git", "log", "-1", "--oneline", h],
                                      capture_output=True, text=True).stdout
                print(f"{'*' if matches(auth[h], idents) else ' '} "
                      f"{subj.rstrip()}  [{auth[h]}]")

        show("Fixes-commits", total, authors)
        if args.all_commits:
            show("Other commits", all_total - total, all_auth)
    return 0


if __name__ == "__main__":
    sys.exit(main())
