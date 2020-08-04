#! /bin/bash
set -e
cd docs/images/tex || exit 1
buildfile=build.tex
retval=0
while read -r -d $'\0' file; do
	[ -e "$file" ] || continue
	echo "Building image '$file'"
	if ! git diff --exit-code "$1^...$1" -- "$file" > /dev/null; then
		cp "gammasyn_doc_images_base.tex" "$buildfile"
		filename=$(basename -- "$file")
		filename="${filename%.*}"
		echo "\addtikzimage{$filename}" >> "$buildfile"
		echo "\end{document}" >> "$buildfile"
		pdflatex -shell-escape -interaction=nonstopmode "$buildfile" || :
		retval=$?
		rm -f "$buildfile"
		if [ ! $retval ]; then
			exit $retval
		fi
	fi
done < <(find ./tikz_src/ -name "*.tikz" -print0)
rm -f "$buildfile"
exit $retval
