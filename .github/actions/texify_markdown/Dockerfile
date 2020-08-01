FROM miktex/miktex:2.9.6990

RUN mpm --admin --verbose --update

COPY texlive.packages /texlive.packages
RUN mpm --admin --verbose --require=@texlive.packages

COPY entrypoint.sh /entrypoint.sh

RUN chmod +x /entrypoint.sh

# Code file to execute when the docker container starts up (`entrypoint.sh`)
ENTRYPOINT ["/entrypoint.sh"]
