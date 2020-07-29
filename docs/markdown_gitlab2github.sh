#! /bin/sh
set -e
if [ -z "$1" ]; then
	targetbranch="$1"
else
	targetbranch=
fi
if [ "${GITHUB_REPOSITORY}" = "" ]; then
	projectname=gammasyn
	username=pvogt09
else
	projectname="${GITHUB_REPOSITORY##*/}"
	username="${GITHUB_REPOSITORY%%/*}"
fi
if [ "${GITHUB_REF#refs/heads/}" = "" ]; then
	if git branch --show-current > /dev/null; then
		branchname=$(git branch --show-current)
	else
		branchname=master
	fi
else
	branchname="${GITHUB_REF#refs/heads/}"
fi
python ./docs/markdown_gitlab2github.py "$(realpath ./)" || exit 1
python -m readme2tex --svgdir docs/svgs --project "$projectname" --username "$username" --output "README.md" "README.tex.md" || exit 2
sed -i "s#https://rawgit.com/$username/$projectname/None#https://raw.githubusercontent.com/$username/$projectname/$branchname#" "README.md"
sed -i "s#https://rawgit.com/$username/$projectname/$branchname#https://raw.githubusercontent.com/$username/$projectname/$branchname#" "README.md"
if [ -z $targetbranch ]; then
	sed -i "s#https://raw.githubusercontent.com/$username/$projectname/$branchname#https://raw.githubusercontent.com/$username/$projectname/$targetbran#" "README.md"
fi
