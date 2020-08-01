FROM danteev/texlive:latest

# RUN mpm --admin --verbose --update

COPY texlive.packages /texlive.packages
# RUN mpm --admin --verbose --require=@texlive.packages
RUN apt-get install texlive-extra-utils || true
# RUN tlmgr init-usertree
# RUN su root -c "tlmgr install $(xargs < /texlive.packages)"

COPY entrypoint.sh /entrypoint.sh

RUN chmod +x /entrypoint.sh

# Code file to execute when the docker container starts up (`entrypoint.sh`)
ENTRYPOINT ["/entrypoint.sh"]
