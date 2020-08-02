FROM danteev/texlive:latest

# RUN mpm --admin --verbose --update

COPY texlive.packages /texlive.packages
# RUN mpm --admin --verbose --require=@texlive.packages
RUN cat /etc/apt/sources.list
RUN apt-get install texlive-binaries -y --install-recommends || true
RUN apt-get install dvisvgm -y --install-recommends || true
RUN tlmgr init-usertree
RUN su root -c "tlmgr install $(xargs < /texlive.packages)" || true
RUN find /usr/share/texlive -name '*dvisvg*' || true
RUN test -r /usr/bin/dvisvgm || true
RUN tlmgr info dvisvgm || true
RUN echo "$PATH"
RUN find /usr/share/texlive -name '*dvisvg*' || true
RUN ls -lah /usr/bin || true
RUN ls -lah /usr/share/texlive/texmf-dist/scripts/texlive-extra || true

COPY entrypoint.sh /entrypoint.sh

RUN chmod +x /entrypoint.sh

# Code file to execute when the docker container starts up (`entrypoint.sh`)
ENTRYPOINT ["/entrypoint.sh"]
