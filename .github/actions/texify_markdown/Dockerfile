FROM danteev/texlive:latest

# RUN mpm --admin --verbose --update

COPY texlive.packages /texlive.packages
# RUN mpm --admin --verbose --require=@texlive.packages
RUN echo "deb http://ftp.debian.org/debian sid main" >> /etc/apt/sources.list || true
RUN apt-get update && apt-get upgrade -y --no-install-recommends || true
# RUN apt-get install texlive-binaries -y --install-recommends || true
# RUN apt-get install dvisvgm -y --install-recommends || true
# RUN su root -c "tlmgr install $(xargs < /texlive.packages)" || true
RUN apt-get install apt-utils -y
RUN apt-get install libwoff1 dvisvgm -y || true
# RUN wget "http://ftp.debian.org/debian/pool/main/d/dvisvgm/dvisvgm_2.9.1-1_amd64.deb" -O /dvisvgm_2.9.1-1_amd64.deb
# RUN dpkg -i /dvisvgm_2.9.1-1_amd64.deb

COPY entrypoint.sh /entrypoint.sh

RUN chmod +x /entrypoint.sh

# Code file to execute when the docker container starts up (`entrypoint.sh`)
ENTRYPOINT ["/entrypoint.sh"]
