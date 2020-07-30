import re
import glob
import os
import sys

if sys.argv and len(sys.argv) > 1:
    directory = sys.argv[1]
else:
    directory = os.path.abspath(r"./../")
os.chdir(directory)

readmes_texmd = glob.glob(r"README.tex.md", recursive=False)
readmes_mdtex = glob.glob(r"README.md.tex", recursive=False)

readme = r""
if len(readmes_texmd) > 1:
    raise Exception('More than one README.tex.md exist')
if len(readmes_mdtex) > 1:
    raise Exception('More than one README.md.tex exist')
if not readmes_texmd:
    if not readmes_mdtex:
        raise Exception('Neither README.tex.md nor README.md.tex exist')
    else:
        readme = readmes_mdtex[0]
else:
    if not readmes_mdtex:
        readme = readmes_texmd[0]
    else:
        raise Exception('Both README.tex.md and README.md.tex exist')
if not readme:
    raise Exception('No readme file found')
readme = directory + os.sep + readme
readme_out = directory + os.sep + 'README.md'

# read file
with open(readme, 'r') as file:
    content = file.read()

# replace content
content = re.sub(
    r'(```math)(.*?)(```)',
    r'$$\2$$',
    content,
    flags=re.MULTILINE | re.DOTALL)
# at minimum one blank line before formula
content = re.sub(
    r'([^\r\n]+[\r\n])(\$\$)(.*?)(\$\$)',
    r'\1\n$$\3$$',
    content,
    flags=re.MULTILINE | re.DOTALL)
content = re.sub(r'(\$`)(.*?)(`\$)', r'$\2$', content)

# write file
with open(readme_out, 'w') as file:
    file.write(content)
