# KURRIER

실행
    roslaunch rosbridge_server rosbridge_websocket.launch
    roslaunch kurrier kurrier.launch

미션
    # 미션 0 시작점
    # 미션 1 차간간격1
    # 미션 2 정적장애물
    # 미션 3 gps음영1(공사장 장애물 회피)
    # 미션 3 gps음영1에서 우회전 후 조금 직진 한 포인트(여기부터 래티스 시작)
    # 미션 4 주차
    # 미션 5 끼어들기2
    # 미션 51 차간간격2        
    # 미션 6 gps음영2(장애물)
    # 미션 61 gps 음영구간 끝나는 곳
    # 미션 7 신호등
    # 미션 71 신호등 사거리 정지선 지점에 멈출수있게 그지점 3미터 전 지점
    # 미션 8 END지점 찾기

할 일
    
    cityhall_path 따기
        미션1 선두차량이 카메라 가운데 나오게(지금 연석 밟으면서 우회전 함)
        미션2 가운데로
        미션3 협로 가운데로 or 협로말고 전체 가운데로
        미션4 가운데로
        미션5 선두 차량이 카메라 가운데에 나오게
        미션6 두번째 슬램 구간 경로 왼쪽으로 붙어서 주행 or 정상적으로 차선 지키며 주행
        미션8 패스 좀 길게 따 놓기(따놓은 패스 안에 홀 커버가 나와야 정지 후 파킹 가능함)

0823

    미션 노드 해당 미션 시작점 가까이가면 그번호로 바뀜 중복가능

0824

    미션 8 마무리

0825

    슬램 시작시 3초 정지후 슬램 키고 시작점 저장하고 다시 3초 대기 후 이동시작
    슬램 끄는거 완료
    SaveFile 압축 풀어서 모라이 런처 폴더 안에 모라이 런처 데이터 안에 SaveFile 폴더 대체 하면 됨

0826

    신호등 완료
    동적장애물 완료
    slam 위치 오차 해결


0828

    주차
    래티스 튜닝
    전부 합치기
    미션2번 너무 늦게 변함

0830

    **우수상**