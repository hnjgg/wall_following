// 인구이동
// BFS

#include<iostream>
#include<queue>

int N,L,R, arr[max][max];
bool visited[max][max];

//BFS탐색에서 인접한 네 방향을 나타내기 위해 사용
int dx[] = {0,0,1,-1}; // 동서남북(0,1)(0,-1)(1,0)(-1,0)
int dy[] = {1,-1,0,0};

queue  <pair<int,int>> q; // pair : 좌표를 나타내기 위해 (x,y)쌍으로

//연합이 생성되면 1. 현재 연합에 포함된 좌표 저장
//               2. 연합 형성 여부 체크 
vector <pair<int,int>> v; 

//queue
void bfs(pair<int,int>start){
    q.push(start);
    visited[start.first][start.second] = true; // 방문표시

    while(!q.empty()){ // queue가 비어있으면 true
        pair<int,int> temp = q.front(); // 가장 앞에 있는 요소 반환
        q.pop;

        for(int i =0; i<4; i++)
        int na = temp.first + dx[i];
        int nb = temp.second + dy[i];

        if(na>=0 && nb>=0 && na<N && nb<N && !visited[na][nb] ){// 유효한 범위(양수, N이하, 방문X)
           if(abs(arr[na][nb] - arr[temp.first][temp.second]) >= L 
                &&
              abs(arr[na][nb] - arr[temp.first][temp.second]) <=R) {
                q.push({na,nb});
                visited[na][nb] = true;

                v.push_back({na,nb});
                sum += arr[na][nb];
              } 
            } 
    }
    
}