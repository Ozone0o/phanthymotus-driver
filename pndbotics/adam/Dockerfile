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

ENV PYTHONPATH=/work
ENV PYTHONUNBUFFERED=1

# 15702 is the driver's MCP endpoint.  The official robot gRPC service is a
# separate process on the NUC and is intentionally not exposed or recreated
# by this image.
EXPOSE 15702

CMD ["/bin/bash", "-lc", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && exec python3 /work/main.py"]
