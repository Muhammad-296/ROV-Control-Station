#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
if [[ ! -f ROV_Control_Station_Integrated.py || ! -f README.md ]]; then echo 'Missing project files'; exit 1; fi
if [[ -d .git ]]; then echo 'Existing Git repository detected. Inspect remotes and history manually.'; exit 1; fi
echo 'Review source for secrets or private data before publishing.'
echo 'Make sure you created an EMPTY GitHub repository named ROV-Control-Station.'
read -r -p 'Have you reviewed files and created the empty repository? Type YES: ' ok
[[ "$ok" == YES ]] || exit 1
git init
git branch -M main
git add .
echo 'Files to be committed:'
git diff --cached --stat
read -r -p 'Commit and push staged files? Type YES: ' ok
[[ "$ok" == YES ]] || exit 1
git commit -m 'Initial release: integrated ROV control station V5'
git remote add origin git@github.com:Muhammad-296/ROV-Control-Station.git
git push -u origin main
