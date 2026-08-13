# PNDbotics Adam Pro — gRPC (高层运控) + DDS (底层状态/灵巧手) driver.
# Reuses the project's standard base image so it is reachable from the build env.
ARG BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${BASE_IMAGE}
ARG PYPI_MIRROR

WORKDIR /work

# protoc is only needed at build time to generate the gRPC stubs from proto/.
RUN apt-get -o Acquire::AllowInsecureRepositories=true update && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
        python3-pip \
        protobuf-compiler && \
    rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/adam-requirements.txt
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r /tmp/adam-requirements.txt grpcio-tools

# Generate adam_control_pb2.py / adam_control_pb2_grpc.py into /work.
COPY proto/ /work/proto/
RUN python3 -m grpc_tools.protoc \
        -I /work/proto \
        --python_out=/work \
        --grpc_python_out=/work \
        /work/proto/adam_control.proto

# Vendored PND DDS SDK (pnd_sdk_python) — source only, cyclonedds comes from pip.
COPY pnd_sdk_python/ /work/pnd_sdk_python/
COPY main.py device.py control.py grpc_client.py dds_client.py ros_bridge.py config.yaml driver.yaml /work/
COPY resource/ /work/resource/

ENV PYTHONUNBUFFERED=1
ENV PYTHONPATH=/work:/work/pnd_sdk_python

EXPOSE 15709

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 /work/main.py"]
