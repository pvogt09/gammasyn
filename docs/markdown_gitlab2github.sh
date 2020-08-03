#! /bin/sh
set -e
if [ -n "$1" ]; then
	targetbranch="$1"
else
	targetbranch=
fi
if [ -z "${GITHUB_REPOSITORY}" ]; then
	projectname=gammasyn
	username=pvogt09
else
	projectname="${GITHUB_REPOSITORY##*/}"
	username="${GITHUB_REPOSITORY%%/*}"
fi
echo "$GITHUB_REF" | grep -qE "^refs/pull/[0-9]+/merge\$" || :
if [ "${GITHUB_REF#refs/heads/}" = "" ] || [ $? ]; then
	if git branch --show-current > /dev/null; then
		branchname=$(git branch --show-current)
	else
		branchname=master
	fi
else
	branchname="${GITHUB_REF#refs/heads/}"
fi
if [ -z "$targetbranch" ]; then
	echo "Creating documentation for branch $branchname"
else
	echo "Creating documentation for PR from ${branchname} to ${targetbranch}"
fi
python ./docs/markdown_gitlab2github.py "$(realpath ./)" || exit 1
python -m readme2tex --svgdir "docs/svgs" --project "$projectname" --username "$username" --output "README.md" "README.tex.md" || exit 2
git checkout -- "README.tex.md" || exit 3
python ./docs/markdown_gitlab2github_replace.py "README.md" "$branchname" "$targetbranch" "$username" "$projectname" || exit 4
