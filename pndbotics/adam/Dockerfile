ARG ADAM_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/drivers/pnpbotics/adam:release.260813.67ce0ec
FROM ${ADAM_BASE_IMAGE}

WORKDIR /work

# The base image supplies the vendor's proprietary pndbotics_sdk_py, ROS 2,
# and CycloneDDS runtime.  Only the driver-side source is replaced here.
COPY config.yaml /work/config.yaml
COPY main.py /work/main.py
COPY device.py /work/device.py
COPY grpc_client.py /work/grpc_client.py
COPY proto/ /work/proto/
COPY resource/ /work/resource/
COPY deploy/ /deploy/

ENV PYTHONUNBUFFERED=1

# The ZED SDK itself stays on the Jetson host (licensed, architecture-specific
# files). deploy/service.yml mounts its libraries and pyzed extension at
# runtime. Pillow is used only to encode the SDK's BGRA frame as JPEG.
ARG APT_MIRROR_HOST=ports.ubuntu.com
RUN sed -i "s|mirrors.tencentyun.com|${APT_MIRROR_HOST}|g" /etc/apt/sources.list \
 && apt-get -o Acquire::AllowInsecureRepositories=true update \
 && apt-get install -y --no-install-recommends --allow-unauthenticated \
      libturbojpeg \
      libusb-1.0-0 \
      libegl1 \
      libv4l-0 \
      libx11-6 \
      libxext6 \
      libbsd0 \
 && rm -rf /var/lib/apt/lists/*

ARG PYPI_MIRROR=https://pypi.org/simple/
RUN python3 -m pip install --no-cache-dir -i ${PYPI_MIRROR} Pillow

ENV PYTHONPATH=/work:/opt/pyzed
ENV ZED_PYTHON_PATH=/opt/pyzed
ENV LD_LIBRARY_PATH=/usr/local/zed/lib:/opt/cyclonedds/lib

# 15702 is the driver's MCP endpoint.  The official robot gRPC service is a
# separate process on the NUC and is intentionally not exposed or recreated
# by this image.
EXPOSE 15702

CMD ["/bin/bash", "-lc", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && exec python3 /work/main.py"]
