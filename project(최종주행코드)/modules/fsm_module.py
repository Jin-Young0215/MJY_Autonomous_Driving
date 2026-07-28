# modules/fsm_module.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

try:
    import afb
    _HAS_AFB = True
except Exception:
    _HAS_AFB = False

from modules.drive_module import (
    BASE_ANGLE, forward, turn_left, turn_right,
    lane_adjust_left, lane_adjust_right, back, stop
)




THETA0 = 3.0
THETA2 = 11.0

# 가속(언덕) 구간 파라미터
BOOST_SPEED = 10
BOOST_TIME  = 1.5   # 초. 필요시 0.8~1.5 사이로 조정

def _drive_by_angle(angle):
    if angle is None:
        stop(); return "STOP"
    e = float(angle) - float(BASE_ANGLE)
    if abs(e) <= THETA0:
        forward(angle=angle); return "FORWARD"        # 직진도 각도 반영
    if e <= -THETA2:
        turn_left(); return "LEFT"
    if e >= THETA2:
        turn_right(); return "RIGHT"
    else:
        forward(angle=angle); return "FORWARD"

class LaneFSM:
    def __init__(self):
        # 상태(state)는 정수로 관리(1~10)
        self.state = 1
        self._t0 = None
        self._seq_done = False

    def reset_timer(self):
        # 상태 시작 시점 기록
        self._t0 = time.time()
        self._seq_done = False

    def elapsed(self):
        if self._t0 is None: return 0.0
        return time.time() - self._t0

    # update는 매 프레임(또는 주기적 호출)마다 호출되어
    # 현재 상태에 따라 동작을 수행하고 필요 시 상태 전이를 한다.
    # angle: 분석된 목표 서보 각도
    # selected_label: 외부에서 수신한 라벨(신호등, 표지판 등)
    # key_start: 키 입력으로 시작 여부 트리거
    def update(self, angle, selected_label, key_start=False):
        s = self.state

        # (원본 코드에 있던 의도치 않은 검사 동일하게 유지)
        if s != self.state:
            stop()
            time.sleep(0.5)

        if s == 1:
            # 초기 상태: 정지 상태, 시작 키를 누르면 다음으로
            stop()
            if key_start:
                self.state = 2
                self.reset_timer()
            return self.state

        if s == 2:
            # 주행 상태: 각도 기반 주행
            _drive_by_angle(angle-5)
            # 특정 라벨(예: smallhill)이 들어오면 부스터 상태로 전이
            if selected_label == "smallhill":
                self.state = 3
                self.reset_timer()
            return self.state

        if s == 3:
            # boost(언덕) 구간: BOOST_TIME 동안 BOOST_SPEED로 전진
            t = self.elapsed()
            if t < 2.5:
                forward(speed=120, angle=angle-5)
            else:
                _drive_by_angle(angle-5)
            
                # 언덕 이후 특정 표지(예: rightsign)를 기다리는 로직
                # (원본 코드의 들여쓰기/구조를 그대로 유지)
                if selected_label == "rightsign":
                    self.state = 4
                    self.reset_timer()
                return self.state
            
        

        if s == 4:
            # rightsign 감지 후 오른쪽 회전을 일정 시간(5초) 수행
            t = self.elapsed()
            if t < 5.0:    
                turn_right()
            else:
                _drive_by_angle(angle-5)
                
                if selected_label == "gateclosed":
                    self.state = 5
                    self.reset_timer()
                return self.state
            


        if s == 5:
            # 문(gate)이 닫혀있으면 정지, 열리면 다음 상태로
            stop()
            if selected_label == "gateopen" or selected_label=="None":
                self.state = 6
                self.reset_timer()
            return self.state
            

        if s == 6:
            # 다시 주행 모드
            t = self.elapsed()
            _drive_by_angle(angle-5)
            
            if selected_label == "staticb":
                self.state = 7
                self.reset_timer()
            return self.state

        if s == 7:
            # staticb 감지 시 왼쪽으로 3초 회전
            t = self.elapsed()
            if t < 3.0:
                turn_left()
            else:
                _drive_by_angle(angle-5)
                
                if selected_label == "staticy":
                        self.state = 8
                        self.reset_timer()
                return self.state

        if s == 8:
            # staticy 감지 전까지 오른쪽으로 3초 회전하고 원상복구
            t = self.elapsed()
            if t < 3.0:
                turn_right()
            else:
                _drive_by_angle(angle-5)
                if selected_label == "leftlight":
                    self.state = 9
                    self.reset_timer()
                return self.state

        if s == 9:
            # 신호 대기 상태(멈춤). 초록 신호나 시간 경과로 진행
            stop()
            t = self.elapsed()
            if selected_label == "greensign" or t > 8.0 :
                self.state = 10
                self.reset_timer()
            return self.state

        if s == 10:
            # 마지막으로 왼쪽 회전(5초) 후 주행으로 복귀
            t = self.elapsed()
            if t < 5.0:
                turn_left()
            else:
                _drive_by_angle(angle-5)
            return self.state
