package tileworld.agent;

import tileworld.Parameters;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWTile;

import java.util.ArrayList;
import java.util.Map.Entry;
import java.util.Arrays;
//import Collectio
import tileworld.environment.TWObstacle;
public class TWAgent1 extends TWAgent{
    protected String name;
    protected int state;
    protected int fuelx;
    protected int fuely;
    protected double fuelTolerance;
    protected int MapSizeX;
    protected int MapSzieY;
//    ~~~~~~~~~~~~~~~ Astar Parameters~~~~~~~~~~~`
    protected ArrayList<int []> openset;
    protected ArrayList<int []> closeset;
    protected int g_score[][];;
    protected int h_score[][];;
    protected int f_score[][];;
    protected int came_from[][][];
//    ~~~~~~~~~~~~~~~~~~water flood~~~~~~~~~~~~~~~~~
    protected int lastpoint[][][];
    protected int distances[][];
    protected ArrayList<int[]> calculatedArea;
    protected ArrayList<int[]> toCalculateArea;
    public TWAgent1(int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env,fuelLevel);
        fuelTolerance=0.9;
        MapSizeX=this.getEnvironment().getxDimension();
        MapSzieY=this.getEnvironment().getyDimension();
        name="agent1";
    }
    protected void initialAstar(int x, int y, int target_x,int target_y){
        ArrayList<int []> openset=new ArrayList<int []>();
        ArrayList<int []> closeset=new ArrayList<int []>();
        openset.add(new int[] {this.getX(),this.getY()});
        g_score= new int[this.MapSizeX][this.MapSzieY];;
        h_score=new int[this.MapSizeX][this.MapSzieY];;
        f_score=new int[this.MapSizeX][this.MapSzieY];;
        came_from=new int[this.MapSizeX][this.MapSzieY][2];
        Arrays.fill(g_score,MapSizeX+MapSzieY);
        Arrays.fill(h_score,MapSizeX+MapSzieY);
        Arrays.fill(f_score,MapSizeX+MapSzieY);
        h_score[x][y]=Math.abs(x-target_x)+Math.abs(y-target_y);
    }

    protected ArrayList<int []> Astar(int x, int y, int target_x,int target_y){
        initialAstar(x,y,target_x,target_y);
        ArrayList<int[]> result_route=new ArrayList<int []>();

        while (openset.size()>0){
            int candidate[]=get_min(openset,f_score);
            if(candidate[0]==target_x && candidate[1]==target_y){
                int node[]={target_x,target_y};
                result_route.add(0,node);
                while(node[0]!=x || node[1]!=y){
                    result_route.add(0,Arrays.copyOf(node,2));
                    node=came_from[node[0]][node[1]];
                }
//                for(int i=-1;i<2;i++) {
//                    for (int j = -1; j < 2; j++) {
//                        if(this.getEnvironment().isInBounds(candidate[0]+i,candidate[1]+j)){
//
//                        }
//                    }
//                }
                return result_route;
            }
            openset.remove(candidate);
            closeset.add(candidate);
            int min_distance=MapSizeX+MapSzieY;
            for(int i=-1;i<2;i++){
                for(int j=-1;j<2;j++){
                    if(i*j!=0||(i==0 && j==0)){
                        continue;
                    }
                    int better_estimate_g=0;
                    if(this.getEnvironment().isInBounds(candidate[0]+i,candidate[1]+j) && !(this.memory.getObjects() instanceof TWObstacle)){
                        int tempindex[]={candidate[0]+i,candidate[1]+j};
                        if(ifListContain(closeset,tempindex)){
                            continue;
                        }
                        else{
                            int estimate_g= g_score[candidate[0]][candidate[1]]+1;
                            if(!ifListContain(openset,tempindex)){
                                better_estimate_g=1;
                            }
                            else if(estimate_g<g_score[tempindex[0]][tempindex[1]]){
                                better_estimate_g=1;
                            }
                            else{
                                better_estimate_g=0;
                            }
                            if (better_estimate_g==1){
                                came_from[tempindex[0]][tempindex[1]]=tempindex;
                                g_score[tempindex[0]][tempindex[1]]=estimate_g;
                                h_score[tempindex[0]][tempindex[1]]=Math.abs(tempindex[0]-target_x)+Math.abs(tempindex[1]-target_y);
                                f_score[tempindex[0]][tempindex[1]]=g_score[tempindex[0]][tempindex[1]]+h_score[tempindex[0]][tempindex[1]];
                                openset.add(new int[] {tempindex[0],tempindex[1]});
                            }
                        }
                    }
                }
            }
        }
        return result_route;
    }
    public ArrayList<int[]> getSurround(ArrayList<int[]> pointList){
        ArrayList<int[]> surroundPoints=new ArrayList<int[]>();
        for(int[] point:pointList){
            for(int i=-1;i<2;i++){
                for(int j=-1;j<2;j++){
                    if(this.getEnvironment().isInBounds(point[0]+i,point[1]+j)&&(i*j==0)&&(i!=0&&j!=0)){
                        if(ifListContain(surroundPoints,new int[]{point[0]+i,point[1]+j})||
                                ifListContain(this.toCalculateArea,new int[]{point[0]+i,point[1]+j})||
                                ifListContain(this.calculatedArea,new int[]{point[0]+i,point[1]+j})){
                            continue;
                        }
                        else{
                            surroundPoints.add(new int[]{point[0]+i,point[1]+j});
                        }
                    }
                }
            }
        }
        return surroundPoints;
    }
    public void waterFlood(int source_x, int source_y){
        this.lastpoint=new int[this.MapSizeX][this.MapSzieY][2];
        this.distances=new int[this.MapSizeX][this.MapSzieY];
        this.calculatedArea=new ArrayList<int[]>();
        this.toCalculateArea=new ArrayList<int[]>();
        Arrays.fill(distances,-1);
        calculatedArea.add(new int[]{source_x,source_y});
        distances[source_x][source_y]=0;
        while(ifArrayContain(distances,-1)){
            this.toCalculateArea=getSurround(this.calculatedArea);
            for(int[] surroundpoint:this.toCalculateArea){
                for(int i=-1;i<2;i++){
                    for(int j=-1;j<2;j++){
                        if(this.getEnvironment().isInBounds(surroundpoint[0]+i,surroundpoint[1]+j)&&(i*j==0)&&(i!=0&&j!=0)){
                            if(ifListContain(this.calculatedArea,new int[]{surroundpoint[0]+i,surroundpoint[1]+j})){
                                distances[surroundpoint[0]][surroundpoint[1]]=distances[surroundpoint[0]+i][surroundpoint[1]+j]+1;
                                this.lastpoint[surroundpoint[0]][surroundpoint[1]][0]=surroundpoint[0]+i;
                                this.lastpoint[surroundpoint[0]][surroundpoint[1]][1]=surroundpoint[1]+j;
                            }
                            else{
                                System.out.println("there is error in calculating surrounding points\n");
                            }
                        }
                        else{
                            continue;
                        }
                    }
                }
            }
            this.calculatedArea.addAll(this.toCalculateArea);
            this.toCalculateArea=new ArrayList<int[]>();
        }
    }
    public ArrayList<int[]> routeByWaterFlood(int source_x,int source_y,int target_x,int target_y){
        int x=target_x;
        int y=target_y;
        ArrayList<int[]> resultRoute=new ArrayList<int[]>();
        resultRoute.add(new int[]{x,y});
        while(x!=source_x||y!=source_y){
            x=lastpoint[target_x][target_y][0];
            y=lastpoint[target_x][target_y][1];
            resultRoute.add(new int[]{x,y});
        }
        return resultRoute;
    }
    public int[] get_min(ArrayList<int[]> array,int[][] scores){
        int min=MapSizeX+MapSzieY;
        int[] minindex= new int[2];
        for(int[] index :array ){
            if(scores[index[0]][index[1]]<min){
                min=scores[index[0]][index[1]];
                minindex=index;
            }
        }
        return minindex;
    }
    public boolean decisde_if_available(int target_x,int target_y){
        if (this.fuelx==-1||this.fuely==-1){
            System.out.println("need to firstly find the fuel station.");
            return true;
        }
        ArrayList<int []> result_route1=Astar(this.getX(),this.getY(),target_x,target_y);
        ArrayList<int []> result_route2=Astar(target_x,target_y,this.fuelx,this.fuely);
        if ((double)result_route1.size()+result_route2.size()<(double)this.getFuelLevel()*this.fuelTolerance){
            return true;
        }
        else{
            return false;
        }
    }

    protected int[] getClosest(Class<?> type){

        for(int i=0;i<this.MapSizeX;i++){
            for(int j=0; j<this.MapSzieY;j++){

            }
        }

    }
    protected TWThought exploration(){
        memory=this.getMemory();


    }
    protected TWThought think(){
//        this.sensor.sense();
        //state 0: exploration
        //state 1: ToTile
        //state 2: ToHole
        //state 3: ToFuel
        if (this.state==0){

        }
    }
    protected boolean ifArrayContain(int [][]distances,int value){
        for(int i=0;i<distances.length;i++){
            for(int j=0;j<distances[0].length;j++){
                if(distances[i][j]==value){
                    return true;
                }
            }
        }
        return false;
    }
    public boolean ifListContain(ArrayList<int[]> arrayList,int[] index){
        for (int i[]: arrayList){
            if(i[0]==index[0]&&i[1]==index[1]){
                return true;
            }
        }
        return false;
    }
}
