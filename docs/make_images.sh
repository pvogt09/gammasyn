#! /bin/bash
set -e
cd docs/images/tex || exit 1
buildfile=build.tex
find ./tikz_src/ -name "*.tikz" -print0 | while read -r -d $'\0' file; do
	[ -e "$file" ] || continue
	echo "Building image '$file'"
	if true || git diff --exit-code HEAD...HEAD^@ -- "$file"; then
		cp "gammasyn_doc_images_base.tex" "$buildfile"
		filename=$(basename -- "$file")
		filename="${filename%.*}"
		echo "\addtikzimage{$filename}" >> "$buildfile"
		echo "\end{document}" >> "$buildfile"
		pdflatex -shell-escape -interaction=nonstopmode "$buildfile" > /dev/null
	fi
done
rm -f "$buildfile"
