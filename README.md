## 자율주행/ADAS Motion Planning & Control (CarMaker, MATLAB & Simulink)

|프로젝트 기간|이름|수행 역할|
|:---:|:---:|:---:|
|2025.05 ~ 2025.06|이강호|Parking, Test Case 작성|  

## 프로젝트 개요  
<img width="300" height="300" alt="Image" src="https://github.com/user-attachments/assets/cb2a4495-70d5-4fc0-a4ef-d438e19a390a" /> <br> <img width="300" height="300" alt="Image" src="https://github.com/user-attachments/assets/59e7edb1-8269-4371-8989-26a4c81aba1b" /> <br>● Detection Data와 Localization 정보를 기반으로 Planning과 Control을 하여 주어진 <br> 시뮬레이션 환경에서 4가지 미션을 수행하는 프로젝트 <br>● 실제 주행 시나리오를 통해 시스템의 작동 원리에 대해 더 깊이 이해

## SW Architecture  
<img width="400" height="450" alt="Image" src="https://github.com/user-attachments/assets/aa6cc58e-4b96-4b90-bf24-7e1ca28f9487" /> <br>

### MATLAB & Simulink Diagram  
<img width="400" height="450" alt="Image" src="https://github.com/user-attachments/assets/5488ec9b-5f3f-4612-b607-962c51c13b34" /> <br> -> FSM을 설계하여 Driving Mode, To-Parking-Area Mode, Parking Mode를 Flag로 전환할 수 있다. <br>

## 1️⃣ Global Path Planning  
Global Path Planning을 위한 경로 탐색 알고리즘 비교 후 A* 알고리즘 적용  
<img width="400" height="450" alt="Image" src="https://github.com/user-attachments/assets/b22a5e98-d87a-470d-bec1-59241248196a" /> <br>
-> 3개의 차선([w1,w2,w3])에 대한 가중치를 부여하여 Trajectory 경로 생성시 사용 <br>

## 2️⃣ Driving  
<table>
  <tr>
    <td style="vertical-align: top; text-align: left;">
      [Local Path Planning]<br>
      1. Catesian to Frenet Frame<br>
      2. Make Path List<br>
      3. Make Obstacle List<br>
      4. Decision Path<br>
      5. Frenet to Catesian Frame<br>
      6. Catesian to GlobalFrame<br><br>
      [Driving Control]<br>
      1. PD Speed Control<br>
      2. Pure Pursuit Steering Control
    </td>
    <td>
      <img width="300" height="300" alt="Path Planning Image" src="https://github.com/user-attachments/assets/a8abf03c-56ef-44e6-8a6c-635378a90760" />
    </td>
  </tr>
  <tr>
    <td style="vertical-align: top; text-align: left;">
      [Trajectory Planning]<br>
      1. Candidate Trajectory Planning<br>
      2. Cost Function Trajectory Planning
    </td>
    <td>
      <img width="300" height="300" alt="Image" src="https://github.com/user-attachments/assets/c977e7ed-eb5e-4915-80f8-4747faed30c1" /><img width="300" height="300" alt="Image" src="https://github.com/user-attachments/assets/3aa4c221-5cf1-436d-b6c3-8c9fa564abc8" />
    </td>
  </tr>
</table>


