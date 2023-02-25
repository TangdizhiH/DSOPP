FROM tandizhihua/dsopp:ros-noetic

USER root

RUN apt-get update \
  && add-apt-repository ppa:neovim-ppa/unstable -y \
  && apt-get update \
  && apt-get install -y neovim \
  && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

USER user

CMD ["/bin/bash"]
