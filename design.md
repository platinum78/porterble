# Porterble 시스템 설계 지침

## 1. 하드웨어 구성
### 1.1. 기계 시스템 구성

### 1.2. 전기 시스템 구성

### 1.3. 전자 / 컴퓨터 시스템 구성

#### SBC / MCU 의 종류 및 역할
* Raspberry Pi
  * 코드명: godfather
  * 중앙 통신 체계
  * 컴퓨터 비전, Kinematics Solver & Control
* Arduino Uno R3
  * 코드명: watchdog
  * 초음파 센서 
* Arduino Mega 2560
  * 코드명: driver
  * 모터 및 엔코더 제어
* Arduino Nano
  * 코드명: torch
  * 조명 시스템
  * Non-blocking 센서 (즉시 응답 가능한 센서)

## 2. 통신 체계

### 2.2. 각 SBC / MCU 간 통신 규약
#### godfather <-> watchdog
* godfather의 요청에만 응답하는 동기적 통신
* 통신 방법: USB 시리얼 통신
* 거리 측정 Request: godfather -> watchdog
  * 측정 및 응답 요청만 전송
  * 2바이트 ASCII 문자열
  * (ENQ: 0x04) + (EOL: \n)
* 거리 측정 Response: watchdog -> godfather
  * 9개의 초음파 센서에 대한 측정값을 밀리미터 단위로 전송
  * 19바이트 ASCII 문자열
  * (센서 측정값: unsigned short) x 9 + (EOL: \n)

#### godfather <-> driver
* godfather의 요청에만 응답하는 동기적 통신
* 속도 설정 Request: godfather -> driver
  * 4개의 모터에 대해 설정할 속도 값을 엔코더 step / sec 단위로 전송
  * 9바이트 ASCII 문자열
  * (모터 속도: signed short) x 4 + (EOL: \n)
* 엔코더 Report: driver -> godfather
  * driver가 측정한 시간의 증분(microseconds)과 4개의 엔코더 값의 변화분을 step 단위로 전송
  * 13바이트 ASCII 문자열
  * (시간 증분: unsigned int) + (엔코더 변화분: signed short) x 4 + (EOL: \n)

#### godfather <-> torch
* godfather의 요청에만 응답하는 동기적 통신
* 조명 점멸 Request: godfather -> torch
  * 조명 장치 코드와 점등/소등 코드 전송 (소등 == 0, 점등 != 0)
  * 2바이트 ASCII 문자열
  * (조명 상태 코드: unsigned char) + (EOL: \n)
    * 조명 상태는 각 비트의 값으로 표현
    * 전조등, 후미등, 좌측 방향지시등, 우측 방향지시등, 비상등, 미정 순서대로 Big Endian으로 전송
* 조명 점멸 Response: torch -> godfather
  * 작동 여부를 전송
  * 2바이트 ASCII 문자열
  * (ACK/NAK: 0x06/0x15) + (EOL: \n)