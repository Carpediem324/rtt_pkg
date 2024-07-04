# rtt_pkg

ROS2 Action Programming

Calculate Round Trip Time on Server - Client

A package that utilizes ROS2's action programming to measure communication speed, latency, and bandwidth between ros - vpn - ros1


## ROS 2

- ROS 2가 기본적으로 설정되어 있는 wifi나 이더넷 네트워크 인터페이스가 아닌 VPN으로 만들어진 tun0를 통해 통신을 하기 위해서는 fast-dds를 위한 profile 파일을 수정하고 이를 환경변수로 등록해주어야 한다.
- In order to communicate through tunnel0 made of VPN, not wifi or Ethernet network interface, where ROS 2 is set by default, the profile file for fast-ds must be modified and registered as an environmental variable.
  
- `ros2_vpn_profiles.xml`
    
    ```xml
    <?xml version="1.0" encoding="UTF-8" ?>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>CustomUDPTransport</transport_id>
                <type>UDPv4</type>
                <interfaceWhiteList>
                    <address>tun0</address>
                </interfaceWhiteList>
            </transport_descriptor>
        </transport_descriptors>
        <participant profile_name="CustomUDPTransportParticipant">
            <rtps>
                <userTransports>
                    <transport_id>CustomUDPTransport</transport_id>
                </userTransports>       
        </participant>
    </profiles>
    ```
    
- 위 파일을 /home/ 경로에 만들어 줌
    - <address>tun0</address> 이 부분이 중요
- 환경 변수 export 해주기

- Create the above file in the /home/ path
- <address>tun0</address> This part is important
- Exporting environment variables
  
    ```bash
    export "FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/ros2_vpn_profiles.xml"
    export "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
    ```
