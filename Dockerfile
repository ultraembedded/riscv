# This Dockerfile sets up an environment to build and test the ultraembedded RISC-V project.
# To run it, mount the project directory into the container, e.g.,
#
# Bash:
# docker run -it --rm -v $(pwd):/project riscv_build
#
# Fish:
# docker build -t riscv_build .
# docker run -it --rm -v (pwd):/project riscv_build

# FROM ubuntu:22.04
# FROM verilator/verilator:latest
FROM verilator/verilator:4.038
# Versions: https://hub.docker.com/r/verilator/verilator
# Use version 4.038, per https://github.com/ultraembedded/riscv/issues/17

# Prepare for non-interactive prompts, esp. for `software-properties-common`.
ENV DEBIAN_FRONTEND=noninteractive

# Install build dependencies.
RUN apt-get update && apt-get install -y \
    build-essential \
    gcc \
    make \
    libelf-dev \
    binutils-dev \
    git \
    sudo \
    wget \
    curl \
    screen \
    locales

# Install SystemC.
# Guide: https://gist.github.com/bagheriali2001/0736fabf7da95fb02bbe6777d53fabf7
# SystemC version 2.3.x is good, per note in the following Makefiles (mentioning version 2.3.3):
    # ./top_tcm_axi/tb/makefile.build_sysc_tb
    # ./top_cache_axi/tb/makefile.build_verilated
WORKDIR /systemc
RUN wget https://accellera.org/images/downloads/standards/systemc/systemc-2.3.3.tar.gz
RUN tar -xvzf systemc-2.3.3.tar.gz
# cd systemc-2.3.3 ; mkdir objdir ; cd objdir
WORKDIR /systemc/systemc-2.3.3/objdir
# export CXX=g++
ENV CXX=g++
RUN ../configure
RUN make -j $(nproc)
RUN make install
WORKDIR /systemc/systemc-2.3.3
RUN rm -rf objdir

# ################################
# ### Optional Fish Section ######
# ################################
# Install fish shell.
# RUN apt-add-repository -y ppa:fish-shell/release-3
# RUN apt-get update
# RUN apt-get install -y fish
RUN apt-get install -y build-essential cmake libpcre2-dev gettext libncurses-dev
WORKDIR /fish-build
RUN wget https://github.com/fish-shell/fish-shell/releases/download/3.7.1/fish-3.7.1.tar.xz
RUN tar -xvf fish-3.7.1.tar.xz
WORKDIR /fish-build/fish-3.7.1
RUN cmake .
RUN make -j $(nproc)
RUN make install
RUN fish --version

# Change the default shell to fish
RUN echo "fish" >> ~/.bashrc
RUN echo "echo 'Fish exited.' && exit" >> ~/.bashrc
# ################################
# ### End Fish Section ###########
# ################################


# Set important environment variables.
ENV SYSTEMC_INCLUDE=/systemc/systemc-2.3.3/include
ENV SYSTEMC_LIBDIR=/systemc/systemc-2.3.3/lib-linux64
ENV SYSTEMC_HOME=/systemc/systemc-2.3.3

ENV C_INCLUDE_PATH=/usr/local/share/verilator/include/:/usr/local/share/verilator/include/vltstd:/systemc/systemc-2.3.3/include
ENV CPLUS_INCLUDE_PATH=/usr/local/share/verilator/include/:/usr/local/share/verilator/include/vltstd:/systemc/systemc-2.3.3/include

# The following folder must contain `libsystemc-2.3.3.so`.
ENV LD_LIBRARY_PATH=/systemc/systemc-2.3.3/lib-linux64

# The following folder must contain `verilated.cpp` file.
ENV VERILATOR_SRC=/usr/local/share/verilator/include

# Set locale variables to avoid warnings.
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8
RUN locale-gen $LANG
RUN locale-gen $LC_ALL

# Create a working directory
WORKDIR /project

# Instructions for building and running the testbench
# Run the testbench using:
# ```
# cd top_tcm_axi/tb
# make
# make run
# ```

# If any environment variables or dependencies are specific to your system, refer to the documentation
# at https://github.com/ultraembedded/riscv

ENTRYPOINT ["/usr/local/bin/fish"]
# ENTRYPOINT ["/bin/bash"]
