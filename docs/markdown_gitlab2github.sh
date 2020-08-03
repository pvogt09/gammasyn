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
echo "$GITHUB_REF" | grep -qE "^refs/pull/[0-9]+/merge\$"
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
sed -i "s#https://rawgit.com/$username/$projectname/None#https://raw.githubusercontent.com/$username/$projectname/$branchname#" "README.md"
sed -i "s#https://rawgit.com/$username/$projectname/$GITHUB_REF#https://raw.githubusercontent.com/$username/$projectname/$branchname#" "README.md"
sed -i "s#https://rawgit.com/$username/$projectname/$GITHUB_HEAD_REF#https://raw.githubusercontent.com/$username/$projectname/$branchname#" "README.md"
sed -i "s#https://rawgit.com/$username/$projectname/$GITHUB_BASE_REF#https://raw.githubusercontent.com/$username/$projectname/$branchname#" "README.md"
sed -i "s#https://rawgit.com/$username/$projectname/$branchname#https://raw.githubusercontent.com/$username/$projectname/$branchname#" "README.md"
if [ -n "$targetbranch" ]; then
	sed -i "s#https://raw.githubusercontent.com/$username/$projectname/$branchname#https://raw.githubusercontent.com/$username/$projectname/$targetbranch#" "README.md"
fi
