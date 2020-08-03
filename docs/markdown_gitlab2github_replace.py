import re
import os
import sys
if sys.argv and len(sys.argv) > 1:
    readme = sys.argv[1]
else:
    raise Exception('No filename given')
if sys.argv and len(sys.argv) > 2:
    branchname = sys.argv[2]
else:
    raise Exception('No branch name given')
if sys.argv and len(sys.argv) > 3:
    targetbranch = sys.argv[3]
else:
    raise Exception('No target branch name given')
if sys.argv and len(sys.argv) > 4:
    username = sys.argv[4]
else:
    raise Exception('No user name given')
if sys.argv and len(sys.argv) > 5:
    projectname = sys.argv[5]
else:
    raise Exception('No project name given')
# read file
with open(readme, 'r') as file:
    content = file.read()

rawgit = 'https://rawgit.com/'
usergit = 'https://raw.githubusercontent.com/'

# replace content
content = content.replace(
    (rawgit + username + '/' + projectname + '/None'),
    (usergit + username + '/' + projectname + '/' + branchname))
content = content.replace(
    (rawgit + username + '/' + projectname + '/' + branchname),
    (usergit + username + '/' + projectname + '/' + branchname))
if os.environ.get('GITHUB_REF'):
    content = content.replace(
        (rawgit + username + '/' + projectname + '/'
            + os.environ['GITHUB_REF']),
        (usergit + username + '/' + projectname + '/' + branchname))
if os.environ.get('GITHUB_HEAD_REF'):
    content = content.replace(
        (rawgit + username + '/' + projectname + '/'
            + os.environ['GITHUB_HEAD_REF']),
        (usergit + username + '/' + projectname + '/' + branchname))
if os.environ.get('GITHUB_BASE_REF'):
    content = content.replace(
        (rawgit + username + '/' + projectname + '/'
            + os.environ['GITHUB_BASE_REF']),
        (usergit + username + '/' + projectname + '/' + branchname))
content = re.sub(
    ('(' +
        re.escape(rawgit + username + '/' + projectname + '/')
        + ')(.*?)(/docs/svgs)'),
    (r'\1' + branchname.replace('\\', r'\\') + r'\3'),
    content,
    flags=re.MULTILINE | re.DOTALL)
content = re.sub(
    ('(' +
        re.escape(usergit + username + '/' + projectname + '/')
        + ')(.*?)(/docs/svgs)'),
    (r'\1' + branchname.replace('\\', r'\\') + r'\3'),
    content,
    flags=re.MULTILINE | re.DOTALL)
if targetbranch:
    content = content.replace(
        (usergit + username + '/' + projectname + '/' + branchname),
        (usergit + username + '/' + projectname + '/' + targetbranch))

# write file
with open(readme, 'w') as file:
    file.write(content)
