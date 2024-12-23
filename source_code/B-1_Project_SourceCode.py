import rclpy
import DR_init
import math

### 삼각수 분해 알고리즘 ###
def decompose_triangle_number(T):
    # 역으로 n 계산
    n = int((-1 + math.sqrt(1 + 8 * T)) // 2)
    
    # n으로 삼각수 검증
    if T == n * (n + 1) // 2:
        return list(range(1, n + 1))[::-1]  # 삼각수 구성 요소 리스트 (역순)
    else:
        return None  # 삼각수가 아님
    

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 250, 150

# Gripper
ON, OFF = 1, 0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

#### 처음 컵을 놓는 위치 (변경 가능) ####
start = [566.104, -229.042, 68.500] ###
#######################################

PICK_CUP_PLACE = [357.458, 77.535, 189.013, 22.973, -179.98, 23.497] # 컵 잡는 위치 (고정)
PICK_CUP_PLACE_APP = [357.458, 77.535, 280.000, 22.973, -179.98, 23.497] # 컵 들어올리는 위치 (고정)

CUP_STANDBY = [420.000, 0.000, 260.000, 22.973, -179.98, 23.497] # 스탠바이 위치 (변경 가능)

# 처음 컵을 놓는 위치인 start 좌표를 가지고 프로세스에서 사용되는 초기 좌표 설정
# (변경 가능 / 업데이트 - update_place_start 함수)
PLACE_START = [start[0], start[1], start[2], 148.899, -179.974, 171.582]
PLACE_START_APP = [None, None, None, 148.899, -179.974, 171.582] # 컵 Approach

CUP_FINAL_POSX = [346.397, 60.000, 89.389, 89.695, -89.996, 89.995] # 마지막 컵 잡는 위치

# 오프셋 (위치 이동 by. 삼각형)
Y_OFFSET = ((-142.042) - (-229.042))
Z_OFFSET = (152.02 - 63.093)
X_OFFSET = Y_OFFSET / 2 * math.sqrt(3)

### 시작 좌표 업데이트 함수 ###
def update_place_start(x, y, z) :
    PLACE_START[0] = x
    PLACE_START[1] = y
    PLACE_START[2] = z


#########################################################################
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("stacking_mission", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    # 필요한 라이브러리 import
    try:
        from DSR_ROBOT2 import (
            set_tool, set_tcp,
            set_digital_output, get_digital_input,
            wait, get_current_posx, get_current_posj,
            release_compliance_ctrl, check_force_condition, task_compliance_ctrl, set_desired_force,
            movej, movel, movesx, movejx,
            DR_FC_MOD_REL, DR_AXIS_Z, DR_MVS_VEL_NONE
        )        
        
        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    set_tool("Tool Weight_RG2")
    set_tcp("RG2_TCP")
    
    JReady = [0, 0, 90, 0, 90, 0]
    
    # Gripper 신호 대기
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            print("Wait for digital input")
            pass

    # Gripper 열기
    def release(status=1):
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait(0.5)
        if status:
           wait_digital_input(1)

    # Gripper 닫기
    def grip(status=1):
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(1.5)
        if status:
           wait_digital_input(1)
        
    # Home 위치 이동 (posj)
    def home_position() :
        # Home 위치에서 시작
        release(0)
        print("[ Move to Home position ]")
        movej(JReady, vel=VELOCITY, acc=ACC)

    # 한층 쌓는 함수 코드
    def stacking_mission(count) :
        de_triangle_number = decompose_triangle_number(count) # 삼각수 분해 (count -> 삼각수)
        
        for layer, layer_len in enumerate(de_triangle_number) :
            for i in range(0, layer_len) :
                print(f"[ 놓는 위치 : {PLACE_START} ]") # Log for debugging
                
                if layer == 0 : movel(PICK_CUP_PLACE, vel=VELOCITY, acc=ACC) # 함수 첫 실행시 컵 잡는 위치 이동
                grip(0) # 잡기
                PICK_CUP_PLACE[2] -= 10 # 컵 잡는 위치 업데이트
        
                # approach값 업데이트 (놓는 위치 기준)
                PLACE_START_APP[0] = PLACE_START[0] # 어프로치 x값 지정
                PLACE_START_APP[1] = PLACE_START[1] # 어프로치 y값 지정
                PLACE_START_APP[2] = PLACE_START[2] + 70 # 어프로치 z값 지정
                
                tmp = [posx(PICK_CUP_PLACE_APP), posx(CUP_STANDBY), posx(PLACE_START_APP), posx(PLACE_START)]
                # 컵 들어 올리기 -> 스탠바이 -> Approach -> 놓는위치 / 이동(movesx)
                movesx(tmp, t=3.5, vel_opt=DR_MVS_VEL_NONE) 
                PICK_CUP_PLACE_APP[2] -= 10 # 컵 빼내는 위치 업데이트
        
                # 순응 제어 / 힘 제어 적용
                task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
                set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                while not check_force_condition(DR_AXIS_Z, max=5):
                    pass

                print("[ 컵이 바닥에 닿았습니다. ]")
        
                release_compliance_ctrl()
                release(0) # 놓기
                
                # Approach -> 스탠바이 -> 컵 잡는 위치 / 이동(movesx)
                movesx([posx(PLACE_START_APP), posx(CUP_STANDBY), posx(PICK_CUP_PLACE)], 
                       t=3.3, vel_opt=DR_MVS_VEL_NONE)
            
                if count == 1 :
                    final_cup()
                    return # 맨 꼭대기 층 작업 후에는 아래의 프로세스를 수행하지 않는다.

                PLACE_START[1] += Y_OFFSET # 층의 한줄 쌓기 위한 y offset 이동

                # 한 열의 최우측에 컵을 놓았을 경우에 스타트 좌표를 업데이트
                if i == layer_len-1 :
                    PLACE_START[0] = start[0] - (X_OFFSET * (layer+1))
                    PLACE_START[1] = start[1] + ((Y_OFFSET / 2) * (layer+1))
                    
        # 다음 층으로 이동하기 위한 스타트 좌표 업데이트
        if count != 1 :
            start[0] -= Y_OFFSET / 2 *(1 / math.sqrt(3))
            start[1] += Y_OFFSET / 2
            start[2] += Z_OFFSET
            update_place_start(start[0], start[1], start[2])

        
    def final_cup() :
        movel(CUP_FINAL_POSX, vel=40, acc=40) # 마지막컵 잡는 위치로 이동
        grip(0)
        CUP_FINAL_POSX[2] += 90
        movel(CUP_FINAL_POSX, vel=80, acc=80) # 컵 들어올리기
        
        # 최종 컵 놓는 위치 기준으로 마지막컵 놓는 위치 조정
        PLACE_START[1] -= 12
        PLACE_START[2] = 385
        PLACE_START[3] = 89.677
        PLACE_START[4] = -89.989
        PLACE_START[5] = -89.998
        
        # 마지막컵 Approach
        movejx(posx(PLACE_START), vel=VELOCITY, acc=ACC, sol=2)
        
        # z 방향 내리기
        PLACE_START[2] -= 85
        movel(PLACE_START, vel=VELOCITY, acc=ACC)
        
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass

        print("[ 마지막 컵을 놓았습니다. ]")
        
        release_compliance_ctrl()
        release(0)
        
        # 작업 마치고 홈위치로 이동
        PLACE_START[1] += 180
        movel(PLACE_START, vel=VELOCITY, acc=ACC)
        home_position()
    
    #########################################
    home_position()
    
    stacking_mission(6)
    stacking_mission(3)
    stacking_mission(1)

    rclpy.shutdown()
    
#########################################
if __name__ == "__main__": 
    main()
