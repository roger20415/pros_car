# From PROS dev image
FROM public.ecr.aws/paia-tech/ros2-humble:dev
ENV ROS2_WS /workspaces
ENV ROS_DOMAIN_ID=1

RUN pip3 install -r requirements.txt
# TODO install dependencies 
# RUN apt install -y packages_to_install

# Build your ROS packages
COPY ./src ${ROS2_WS}
WORKDIR ${ROS2_WS}
RUN . /opt/ros/humble/setup.sh && colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
# You could decide wheather to delete source code

RUN echo "source ${ROS2_WS}/install/setup.bash " >> /.bashrc 

COPY ros_entrypoint.bash /ros_entrypoint.bash
RUN chmod +x /ros_entrypoint.bash
ENTRYPOINT [ "/ros_entrypoint.bash" ]
CMD ["bash","-l"]