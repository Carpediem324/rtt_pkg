# rtt_pkg

ROS2 Action Programming

Calculate Round Trip Time on Server - Client

A package that utilizes ROS2's action programming to measure communication speed, latency, and bandwidth between ros - vpn - ros1

## 왜 서비스가 아니라 액션을 사용했는가?

### 한국어(Korean)
1. **장시간 작업에 대한 중간 피드백(Feedback) 필요**  
   - 서비스(Service)는 요청 후 단 한 번의 응답만 받습니다.  
   - 액션(Action)은 긴 작업 중간에도 피드백을 받아 진행 상황을 확인할 수 있습니다.

2. **작업 취소(Cancel) 기능**  
   - 서비스는 호출하면 응답을 받기 전까지 작업을 멈추기 어렵습니다.  
   - 액션은 목표(Goal)를 취소할 수 있어서, 긴 작업 도중에도 유연하게 중단할 수 있습니다.

3. **확장성**  
   - 액션은 Goal-Feedback-Result 구조로, 향후 더 복잡한 로직을 손쉽게 추가할 수 있습니다.  
   - 예컨대 로봇 이동, 대규모 데이터 처리 등에서 유리합니다.

---

### English
1. **Need for intermediate feedback (Feedback) in long-duration tasks**  
   - A service only provides a single response after the request.  
   - An action allows receiving feedback during lengthy operations to monitor progress in real time.

2. **Cancellation capability**  
   - With services, you can’t easily stop the operation once requested until it responds.  
   - Actions let you cancel a goal midway, offering flexibility for lengthy or large-scale tasks.

3. **Scalability**  
   - The Goal-Feedback-Result structure of actions allows for easy expansion of complex logic.  
   - Useful for robot navigation, big data processing, and other time-consuming processes.


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



### Interface pkg 생성

1. Create package
    
    ```python
    ros2 pkg create custom_interfaces2 --build-type ament_cmake --dependencies action_msgs std_msgs rosidl_default_generators
    ```
    

1. action생성
    
    ```python
    mkdir -p custom_interfaces2/action/
    ```
    
2. action 작성 Rtt.action
    
    ```python
    # 요청 (Goal)
    builtin_interfaces/Time request_time
    ---
    # 결과 (Result)
    builtin_interfaces/Time response_time
    ---
    # 피드백 (Feedback)
    builtin_interfaces/Time partial_response_time
    ```
    
3. 
4. CMakeLists.txt수정

```python
set(action_files
    "action/Rtt.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${action_files}
    DEPENDENCIES action_msgs std_msgs
)
```

1. **package.xml 수정**

```python
<depend>builtin_interfaces</depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

```jsx
rosdep install --from-paths src --ignore-src -r -y
```
