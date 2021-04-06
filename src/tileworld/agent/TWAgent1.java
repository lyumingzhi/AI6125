package tileworld.agent;

import sim.field.grid.ObjectGrid2D;
import sim.util.Int2D;
import tileworld.Parameters;
import tileworld.environment.*;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;
import java.util.Map.Entry;
import java.util.Arrays;
import java.util.Random;

//import Collectio
import tileworld.exceptions.CellBlockedException;

//enum State {
//    GET_FUEL,
//    GET_TILE,
//    GET_HOLE,
//    EXPLORE
//}
public class TWAgent1 extends TWAgent{
    protected String name;
    protected State state;
    protected int fuelx=-1;
    protected int fuely=-1;
    protected double fuelTolerance;
    protected int MapSizeX;
    protected int MapSizeY;
    protected int DownStep=0;
    protected int DStep=0;
    protected int UpStep=0;
    protected int UStep=0;
    protected int LeftStep=0;
    protected int RightStep=0;
    protected int flag=0;
    //~~~~~~~~~~~naive~~~~~~
    protected int senseRange=3;
    protected TWFuelStation fuelStation;
    private TWTile targetTile;
    private TWHole targetHole;
    private ArrayList<TWAgent> otherAgents;
//    ~~~~~~~~~~~~~~~ Astar Parameters~~~~~~~~~~~`
    protected ArrayList<int []> openset;
    protected ArrayList<int []> closeset;
    protected int g_score[][];;
    protected int h_score[][];;
    protected int f_score[][];;
    protected int came_from[][][];
//    ~~~~~~~~~~~~~~~~~~water flood~~~~~~~~~~~~~~~~~
    protected int lastPointMap[][][];
    protected int distances[][];
    protected ArrayList<int[]> calculatedArea;
    protected ArrayList<int[]> toCalculateArea;
    protected int checkRange=20;
    protected int initalPosition[][] = new int[1][2];
//    protected int initalPosition[][] = new int[1][4];
    public TWAgent1(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env,fuelLevel);
        fuelTolerance=0.9;
        MapSizeX=this.getEnvironment().getxDimension();
        MapSizeY=this.getEnvironment().getyDimension();
        this.name=name;
        this.otherAgents = new ArrayList<TWAgent>();
        this.state=State.GET_FUEL;


    }

    @Override
    public String getName() {
        return this.name;
    }

    protected void initialAstar(int x, int y, int target_x, int target_y){
        this.openset=new ArrayList<int []>();
        this.closeset=new ArrayList<int []>();
        this.openset.add(new int[] {this.getX(),this.getY()});
        this.g_score= new int[this.MapSizeX][this.MapSizeY];;
        this.h_score=new int[this.MapSizeX][this.MapSizeY];;
        this.f_score=new int[this.MapSizeX][this.MapSizeY];;
        this.came_from=new int[this.MapSizeX][this.MapSizeY][2];
        for(int i=0;i<g_score.length;i++){
            Arrays.fill(g_score[i],MapSizeX+MapSizeY);
            Arrays.fill(h_score[i],MapSizeX+MapSizeY);
            Arrays.fill(f_score[i],MapSizeX+MapSizeY);
        }
        this.g_score[x][y]=0;
        this.h_score[x][y]=Math.abs(x-target_x)+Math.abs(y-target_y);
        this.f_score[x][y]=this.h_score[x][y]+this.g_score[x][y];
    }

    protected ArrayList<int []> Astar(int x, int y, int target_x,int target_y){
        initialAstar(x,y,target_x,target_y);
        ArrayList<int[]> result_route=new ArrayList<int []>();
        int count=0;
        while (this.openset.size()>0){
            count+=1;
//            System.out.println(count+" i am here,"+ this.openset.size());
            int candidate_index =get_min(this.openset,this.f_score);
            int []candidate=this.openset.get(candidate_index);
            if(candidate[0]==target_x && candidate[1]==target_y){
                int node[]={target_x,target_y};
                result_route.add(0,node);
                while(node[0]!=x || node[1]!=y){
//                    result_route.add(0,Arrays.copyOf(node,2));
                    result_route.add(0,node);
                    node=this.came_from[node[0]][node[1]];

                }
//                for(int i=-1;i<2;i++) {
//                    for (int j = -1; j < 2; j++) {
//                        if(this.getEnvironment().isInBounds(candidate[0]+i,candidate[1]+j)){
//
//                        }
//                    }
//                }
//                System.out.println("the distance to FuelStation is "+result_route.size()+" "+result_route.get(0)[0]+" "+result_route.get(0)[1]);
                return result_route;
            }
            this.openset.remove(candidate_index);
//            System.out.println("openset size after remove "+this.openset.size());
            this.closeset.add(candidate);
            int min_distance=MapSizeX+MapSizeY;
            for(int i=-1;i<2;i++){
                for(int j=-1;j<2;j++){
                    if(i*j!=0||(i==0 && j==0)){
                        continue;
                    }
                    int better_estimate_g=0;
                    if(this.getEnvironment().isInBounds(candidate[0]+i,candidate[1]+j) &&
                            (!(this.memory.getMemoryGrid().get(candidate[0]+i,candidate[1]+j) instanceof TWObstacle))){
                        int tempindex[]={candidate[0]+i,candidate[1]+j};
                        if(ifListContain(this.closeset,tempindex)){
                            continue;
                        }
                        else{
                            int estimate_g= this.g_score[candidate[0]][candidate[1]]+1;
                            if(!ifListContain(this.openset,tempindex)){
                                better_estimate_g=1;
                            }
                            else if(estimate_g<this.g_score[tempindex[0]][tempindex[1]]){
                                better_estimate_g=1;
                            }
                            else{
                                better_estimate_g=0;
                            }
                            if (better_estimate_g==1){
                                this.came_from[tempindex[0]][tempindex[1]]=candidate;
                                this.g_score[tempindex[0]][tempindex[1]]=estimate_g;
                                this.h_score[tempindex[0]][tempindex[1]]=Math.abs(tempindex[0]-target_x)+Math.abs(tempindex[1]-target_y);
                                this.f_score[tempindex[0]][tempindex[1]]=g_score[tempindex[0]][tempindex[1]]+h_score[tempindex[0]][tempindex[1]];
                                this.openset.add(new int[] {tempindex[0],tempindex[1]});
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
                    if(this.checkCheckable(point[0]+i,point[1]+j)&&(i*j==0)&&(i!=0||j!=0)){
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
//        System.out.println("surround point: "+surroundPoints.size());
        return surroundPoints;
    }
    public void waterFlood(int source_x, int source_y){
        this.lastPointMap=new int[this.MapSizeX][this.MapSizeY][2];
        this.distances=new int[this.MapSizeX][this.MapSizeY];
        this.calculatedArea=new ArrayList<int[]>();
        this.toCalculateArea=new ArrayList<int[]>();
        for(int i=0;i<this.distances.length;i++){
            Arrays.fill(distances[i],-1);
        }

        calculatedArea.add(new int[]{source_x,source_y});
        distances[source_x][source_y]=0;
        int floodRange=0;
        int ifFindTile=0;
        int ifFindHole=0;
        int ifFindFuelStation=0;
        while(ifArrayContain(distances,-1) ){
//            System.out.println("it is in waterflood\n");
            this.toCalculateArea=getSurround(this.calculatedArea);
            int tempDistances[][]=new int[this.MapSizeX][this.MapSizeY];
            for (int i=0;i<this.MapSizeX;i++){
                for(int j=0;j<this.MapSizeY;j++){
                    tempDistances[i][j]=distances[i][j];
                }
            }
//            if(this.calculatedArea.size()==1){
//                for(int [] i:this.toCalculateArea){
//                    System.out.println("ssssssss "+i[0]+" "+i[1]);
//                }
//            }
            for(int[] surroundpoint:this.toCalculateArea){
                int ifchange=0;
                for(int i=-1;i<2;i++){
                    for(int j=-1;j<2;j++){
                        if(ifchange==1){
                            break;
                        }
                        if(this.checkCheckable(surroundpoint[0]+i,surroundpoint[1]+j)&&(i*j==0)&&(i!=0||j!=0)){
//                            System.out.println("to and calculatedArea: "+this.toCalculateArea.size()+" "+this.calculatedArea.size());
//                            if((this.memory.getMemoryGrid().get(surroundpoint[0]+i,surroundpoint[1]+j) instanceof TWObstacle)
//                                    && (surroundpoint[0]+i==this.getX() && surroundpoint[1]+j==this.getY())){
//                                System.out.println("agent is on an obstacle");
//                                System.exit(0);
//                            }
//                            System.out.println("i: "+i+" j :"+j);
                            if(ifListContain(this.calculatedArea,new int[]{surroundpoint[0]+i,surroundpoint[1]+j})){
                                this.distances[surroundpoint[0]][surroundpoint[1]]=distances[surroundpoint[0]+i][surroundpoint[1]+j]+1;
                                this.lastPointMap[surroundpoint[0]][surroundpoint[1]][0]=surroundpoint[0]+i;
                                this.lastPointMap[surroundpoint[0]][surroundpoint[1]][1]=surroundpoint[1]+j;
                                ifchange=1;
                            }
//
                        }
                    }
                    if(ifchange==1){
                        break;
                    }

                }
                if(ifchange==0){
                    System.out.println("to and calculatedArea: "+this.toCalculateArea.size()+" "+this.calculatedArea.size());
                    System.out.println(" the start point "+this.calculatedArea.get(0)[0]+" "+this.calculatedArea.get(0)[1]);
                    System.out.println("the surround point 1 "+ this.toCalculateArea.get(0)[0]+" "+this.toCalculateArea.get(0)[1]);
                    this.displayMap(this.distances);
                    System.out.println("there is error in calculating surrounding points\n");
                }
                if(this.memory.getMemoryGrid().get(surroundpoint[0],surroundpoint[1]) instanceof TWTile
                        && ((TWTile)(this.memory.getMemoryGrid().get(surroundpoint[0],surroundpoint[1]))).getTimeLeft(this.getMemory().schedule.getTime())>=this.distances[surroundpoint[0]][surroundpoint[1]] ){
                    ifFindTile=1;
                }
                if(this.memory.getMemoryGrid().get(surroundpoint[0],surroundpoint[1]) instanceof TWHole
                        && ((TWHole)(this.memory.getMemoryGrid().get(surroundpoint[0],surroundpoint[1]))).getTimeLeft(this.getMemory().schedule.getTime())>=this.distances[surroundpoint[0]][surroundpoint[1]] ){
                    ifFindHole=1;
                }
                if(surroundpoint[0]==this.fuelx &&surroundpoint[1]==this.fuely){
                    ifFindFuelStation=1;
                }
            }
            this.calculatedArea.addAll(this.toCalculateArea);
            this.toCalculateArea=new ArrayList<int[]>();
            int ifchange1=0;
            for (int i=0;i<this.MapSizeX;i++){
                for(int j=0;j<this.MapSizeY;j++){
                    if(tempDistances[i][j]!=distances[i][j]){
                        ifchange1=1;
                        break;
                    }
                }
                if(ifchange1==1){
                    break;
                }
            }
            floodRange+=1;
//            System.out.println("if change: "+ifchange1+" iffindtile "+ifFindTile+" iffindhole "+ifFindHole+" iffindfuelstation "+ifFindFuelStation+" floodRange "+floodRange);
            if(ifchange1!=1|| ((floodRange==this.checkRange || (ifFindTile==1 && ifFindHole==1 ))) ){
                break;
            }
        }
    }
    public Boolean checkCheckable(int aimx, int aimy) {
        for (TWAgent otherAgent : this.otherAgents) {
            if (this.getEnvironment().isInBounds(aimx, aimy)
                    && (!(this.memory.getMemoryGrid().get(aimx, aimy) instanceof TWObstacle)
                    || (aimx == this.getX() && aimy == this.getY() || (aimx == otherAgent.getX() && aimy == otherAgent.getY())))) {
                return true;
            }
        }
        return false;
    }
    public ArrayList<int[]> routeByWaterFlood(int source_x,int source_y,int target_x,int target_y){
        int x=target_x;
        int y=target_y;
        ArrayList<int[]> resultRoute=new ArrayList<int[]>();
        resultRoute.add(new int[]{x,y});
        while(x!=source_x||y!=source_y){
            x=lastPointMap[target_x][target_y][0];
            y=lastPointMap[target_x][target_y][1];
            resultRoute.add(new int[]{x,y});
        }
        return resultRoute;
    }
    public int get_min(ArrayList<int[]> array,int[][] scores){
        int min=MapSizeX+MapSizeY+1;
        int minindex=-1;
        for(int index=0;index<array.size();index++ ){
            if(scores[array.get(index)[0]][array.get(index)[1]]<min){
                min=scores[array.get(index)[0]][array.get(index)[1]];
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

    protected Object getClosest(Class Type){
        int minDistance=this.MapSizeX+this.MapSizeY;
        int minx=-1;
        int miny=-1;
//        this.displayMap(this.distances);
        for(int i=0;i<this.MapSizeX;i++){
            for(int j=0; j<this.MapSizeY;j++){
                if(this.distances[i][j]!=-1&&this.distances[i][j]<minDistance && Type.isInstance( this.memory.getMemoryGrid().get(i,j))
                        && ((TWObject)(this.memory.getMemoryGrid().get(i,j))).getTimeLeft(this.getMemory().schedule.getTime())>=this.distances[i][j] ){
                    minDistance=this.distances[i][j];
                    minx=i;
                    miny=j;
                }
            }
        }
        if(minx!=-1&&miny!=-1) {
            System.out.println(Type+" the mini x is "+minx+" "+miny+"\n");
            System.out.println("this agent is at "+this.getX()+" "+this.getY());
            System.out.println("this object is "+ this.memory.getMemoryGrid().get(minx,miny).getClass());

//            System.out.println("this distance: "+this.distances);
            //            System.exit(0);
            return this.memory.getMemoryGrid().get(minx, miny);

        }
        else{
            System.out.println("can not find any "+Type);
            return null;
        }
    }
//    protected TWThought exploration(){
//        memory=this.getMemory();
//    }
    protected void displayMap(int [][]map){
        for(int i=0;i<map.length;i++){
            for(int j=0;j<map[i].length;j++){
                System.out.print(map[j][i]+" ");
            }
            System.out.print("\n");
        }
    }
    protected TWThought think(TWThought lastThought){
//        this.sensor.sense();
        //state 0: exploration
        //state 1: ToTile
        //state 2: ToHole
        //state 3: ToFuel
        System.out.println(" Simple Score: " + this.score+" name "+this.name+ " state: "+this.state+" feul level "+this.fuelLevel+"~~~~~~~~~~~~~~~~~~~~~~~~~~~");
        // return new TWThought(TWAction.MOVE, getRandomDirection());
        TWThought thought;

//        this.fuelx=this.getEnvironment().getFuelingStation().getX();
//        this.fuely=this.getEnvironment().getFuelingStation().getY();///need to change after inventing the algorithm to find fuel

        //firstly communicate with other agents and update memory
        this.communicate();
        this.getMemory().updateMemorWithCommunication(this.otherAgents);
        this.checkFuelStation();
        this.waterFlood(this.getX(),this.getY());
        int NumAgents = this.otherAgents.size()+1;
        int avgInterval = (int) Math.ceil(Parameters.yDimension/ NumAgents);
        int rank = 0;
        for(TWAgent otheragent : this.otherAgents){
            if(otheragent.getY()<y){ rank++; }
            else if(otheragent.getY() == y){
                if(otheragent.getX()<x){ rank++; }
            }
        }
        this.initalPosition[0][0] = avgInterval*rank;
        if(avgInterval*(rank+1)>Parameters.yDimension){
            this.initalPosition[0][1] = Parameters.yDimension;
        }else {
            this.initalPosition[0][1] = avgInterval*(rank + 1)+1;
        }
        if(y>this.initalPosition[0][1]){
            int s = this.initalPosition[0][0];
            this.initalPosition[0][0] = this.initalPosition[0][1];
            this.initalPosition[0][1] = s;
        }

//        if(this.initalPosition[0][0] == 0 && this.initalPosition[0][1] == 0){
//            this.initalPosition[0][0] = x;
//            this.initalPosition[0][1] = y;
//            for(TWAgent otheragent : this.otherAgents){
//                if(otheragent.getY() >= this.initalPosition[0][1]){
//                    this.initalPosition[0][0] = otheragent.getX();
//                    this.initalPosition[0][1] = otheragent.getY();
//                }
//            }
//            if(this.initalPosition[0][0] == x && this.initalPosition[0][1] == y){
//                this.initalPosition[0][0] = Parameters.xDimension;
//                this.initalPosition[0][1] = Parameters.yDimension;
//            }
//        }
//        for(TWAgent otheragent:this.otherAgents){
//
//        }
//        displayMap(this.distances);

//        if( this.state!= State.GET_HOLE &&this.getMemory().getMemoryGrid().get(this.getX(),this.getY())instanceof TWHole && this.carriedTiles.size()>0){
//            TWThought think_btw= new TWThought(TWAction.PUTDOWN,TWDirection.Z);
//            think_btw.setHole((TWHole)this.getMemory().getMemoryGrid().get(this.getX(),this.getY()));
//            return think_btw;
//        }
//        if(this.state!= State.GET_TILE && this.getMemory().getMemoryGrid().get(this.getX(),this.getY())instanceof TWTile && this.carriedTiles.size()<3){
//            TWThought think_btw=new TWThought(TWAction.PICKUP,TWDirection.Z);
//            think_btw.setTile((TWTile) this.getMemory().getMemoryGrid().get(this.getX(),this.getY()));
//            return think_btw;
//        }
        switch (this.state) {
            case EXPLORE:
                thought = this.getExploreThought(lastThought);
                this.state = State.GET_TILE;
                break;
            case GET_FUEL:
                thought = this.getFuelThought(lastThought);
                if (thought.getAction() == TWAction.REFUEL) {
                    this.state = State.GET_TILE;
                }
                break;
            case GET_HOLE:
                thought = this.getHoleThought(lastThought);
                if (thought.getAction() == TWAction.PUTDOWN) {
                    this.state = State.GET_TILE;
                }
                break;
            case GET_TILE:
                thought = this.getTileThought(lastThought);
                if (thought.getAction() == TWAction.PICKUP) {
                    this.state = State.GET_HOLE;
                }
                break;
            default:
                thought = this.getExploreThought(lastThought);
                break;
        }
        // No fuel
        if (this.getFuelLevel() < this.getFuelDistance() + 20) {
            this.state = State.GET_FUEL;
        }
        return thought;
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
//        if(arrayList.size()==1)System.out.println("i am in ifLisContain");
        for (int i[]: arrayList){
//            if(arrayList.size()==1)System.out.println("input list and index "+i[0]+" "+i[1]+" "+index[0]+" "+index[1]);
            if(i[0]==index[0]&&i[1]==index[1]){
                return true;
            }
        }
        return false;
    }

    private void beforeCommunicate() {
        this.otherAgents.clear();
    }

    public void communicate() {
        this.beforeCommunicate();
        Message message = new Message(this.getName(),"","", this);
        this.getEnvironment().receiveMessage(message);

        // Receive messages
        ArrayList<Message> messages = this.getEnvironment().getMessages();

        for (int i = 0; i < messages.size(); i++){
            if (messages.get(i).getAgent().getName() == this.getName())
                continue;
            this.otherAgents.add(messages.get(i).getAgent());
        }
    }
    @Override
    protected void act(TWThought thought) {
        try {
            switch (thought.getAction()) {
                case MOVE:
                    this.move(thought.getDirection());
                    break;
                case PICKUP:
                    this.pickUpTile(thought.getTile());
                    break;
                case REFUEL:
                    this.refuel();
                    break;
                case PUTDOWN:
                    this.putTileInHole(thought.getHole());
                    break;
            }
        } catch (CellBlockedException ex) {
            // Cell is blocked, replan?
        }
    }
    private TWDirection getOneStepDirectionToFuelStation(){
        this.initialAstar(this.getX(),this.getY(),this.fuelx,this.fuely);
        ArrayList<int[]> path=this.Astar(this.getX(),this.getY(),this.fuelx,this.fuely);
        int[] nextStep=path.get(0);
        assert ((nextStep[0]-this.getX())*(nextStep[1]-this.getY())==0);
        int x=nextStep[0];
        int y=nextStep[1];
        if (x > this.getX()) {
            return TWDirection.E;
        } else if (x < this.getX()) {
            return TWDirection.W;
        } else { // x found
            if (y > this.getY()) {
                return TWDirection.S;
            } else if (y < this.getY()) {
                return TWDirection.N;
            } else {
                System.out.println("refuel index "+this.fuelx+" "+this.fuely+" the agent is at "+this.getX()+" "+this.getY());
                return TWDirection.Z;
            }
        }
    }
    private TWDirection getOneStepDirection(int x, int y) {

        int initialx=x;
        int initialy=y;
        int []lastpoint=this.lastPointMap[x][y];
        while(!(lastpoint[0]==this.getX()&& lastpoint[1]==this.getY())&&!(x==this.getX()&&y==this.getY())){
             x=lastpoint[0];
             y=lastpoint[1];
             lastpoint=this.lastPointMap[x][y];
        }
        assert ((x-this.getX())*(y-this.getY())==0);
        if (x > this.getX()) {
            return TWDirection.E;
        } else if (x < this.getX()) {
            return TWDirection.W;
        } else { // x found
            if (y > this.getY()) {
                return TWDirection.S;
            } else if (y < this.getY()) {
                return TWDirection.N;
            } else {
                System.out.println("pick up index "+initialx+" "+initialy+" the agent is at "+this.getX()+" "+this.getY());
                return TWDirection.Z;
            }
        }
    }


//    private TWThought getExploreThought(TWThought lastThought) {
//        TWDirection newDir = TWDirection.E;
//        int stopX = initalPosition[0][0];
//        int stopY = initalPosition[0][1];
//        double t = Math.random();
//        double time = this.getMemory().schedule.getTime();
//        Object temp = this.memory.getMemoryGrid().get(x, y);
//        TWAction action = TWAction.MOVE;
//        if(temp instanceof TWTile && this.carriedTiles.size()<3){
////            if(this.getEnvironment().canPickupTile((TWTile) temp, this)) {
////                action = TWAction.PICKUP;
////            }
//            action = TWAction.PICKUP;
//        } else if(temp instanceof TWHole && this.carriedTiles.size()>0) {
//            action = TWAction.PUTDOWN;
//        }
//        if (t < 0.00) {
//            newDir = getRandomDirection();
//        } else {
//            if (lastThought.getDirection() == TWDirection.S) {// 上一步是向下
//                if (y + Parameters.defaultSensorRange >= stopY) {
//                    newDir = TWDirection.N;
//                    DownStep = 0;
//                } else if (DownStep == 7 || y + Parameters.defaultSensorRange >= Parameters.yDimension-1) { //如果已经向下7步或者到达了最低
//                    if (x - Parameters.defaultSensorRange <= 0) {  //判断是都在左边界
//                        newDir = TWDirection.E;
//                    } else if (x + Parameters.defaultSensorRange >= Parameters.xDimension-1) { //判断是否在右边界
//                        newDir = TWDirection.W;
//                    }
//                    DownStep = 0;
//                } else if (DStep == 1) {
//                    newDir = TWDirection.E;
//                    DStep = 0;
//                } else if (DownStep < 7) {  //向下不足7步，继续向下
//                    Object n = this.memory.getMemoryGrid().get(x, y + 1);
//                    if (n instanceof TWObstacle && ((TWObstacle) n).getTimeLeft(time)>3) {
//                        newDir = TWDirection.W;
//                        LeftStep++;
////                    } else if(n instanceof TWTile && this.carriedTiles.size()<3){
////                        action = TWAction.PICKUP;
////                    } else if(n instanceof TWHole && this.carriedTiles.size()>0) {
////                        action = TWAction.PUTDOWN;
//                    } else {
//                        newDir = TWDirection.S;
//                        DownStep++;
//                    }
//                }
//
//            } else if (lastThought.getDirection() == TWDirection.E) { //上一步是向右
//                if (y + Parameters.defaultSensorRange >= stopY) {
//                    newDir = TWDirection.N;
//                    RightStep = 0;
//                } else if (RightStep == 1) {
//                    newDir = TWDirection.N;
//                    RightStep = 0;
//                } else if (x + Parameters.defaultSensorRange >= Parameters.xDimension-1) { //判断是到右边界
//                    if (y + Parameters.defaultSensorRange >= Parameters.yDimension-1) { //判断是到下边界
//                        newDir = TWDirection.N;
//                    } else {
//                        newDir = TWDirection.S;
//                        DownStep++;
//                    }
//                } else { //没到右边界，继续向右
//                    Object n = this.memory.getMemoryGrid().get(x+1, y);
//                    if (n instanceof TWObstacle && ((TWObstacle) n).getTimeLeft(time)>3) {
//                        newDir = TWDirection.S;
//                        DStep++;
//
//                    } else {
//                        newDir = TWDirection.E;
//                    }
//                }
//            } else if (lastThought.getDirection() == TWDirection.W) { //上一步是向左
//                if (y + Parameters.defaultSensorRange >= stopY) {
//                    newDir = TWDirection.N;
//                } else if (LeftStep == 1) {
//                    newDir = TWDirection.S;
//                    LeftStep = 0;
//                } else if (x - Parameters.defaultSensorRange <= 0) { //判断是到左边界
//                    if (y + Parameters.defaultSensorRange >= Parameters.yDimension-1) { //判断是到下边界
//                        newDir = TWDirection.N;
//                    } else {
//                        newDir = TWDirection.S;
//                        DownStep++;
//                    }
//                } else { //没左到边界，继续向左
//                    Object n = this.memory.getMemoryGrid().get(x-1, y);
//                    if (n instanceof TWObstacle && ((TWObstacle) n).getTimeLeft(time)>3) {
//                        newDir = TWDirection.N;
//                        UpStep++;
//                    } else {
//                        newDir = TWDirection.W;
//                    }
//                }
//            } else { //上一步是向上
//                if (y - Parameters.defaultSensorRange <= 0) { //判断是否到了上边界
//                    if (x - Parameters.defaultSensorRange <= 0) { //判断是到左边界
//                        newDir = TWDirection.E;
//                    } else if (x + Parameters.defaultSensorRange >= Parameters.xDimension-1) { //判断是到右边界
//                        newDir = TWDirection.W;
//                    } else {
//                        newDir = TWDirection.W;
//                    }
//                } else if (UpStep == 1) {
//                    newDir = TWDirection.W;
//                    UpStep = 0;
//                } else {
//                    Object n = this.memory.getMemoryGrid().get(x, y-1);
//                    if (n instanceof TWObstacle && ((TWObstacle) n).getTimeLeft(time)>3) {
//                        newDir = TWDirection.E;
//                        RightStep++;
//                    } else {
//                        newDir = TWDirection.N;
//                    }
//                }
//            }
//        }
//        TWThought tt = new TWThought(action, newDir);
//        if(action==TWAction.PICKUP){
//            tt.setTile((TWTile) temp);
//        } else if (action==TWAction.PUTDOWN){
//            tt.setHole((TWHole) temp);
//        }
//        return tt;
//    }


    private TWThought getExploreThought(TWThought lastThought) {
        TWDirection newDir = TWDirection.E;
        int startY = initalPosition[0][0];
        int stopY = initalPosition[0][1];
        double t = Math.random();
        double time = this.getMemory().schedule.getTime();
        Object temp = this.memory.getMemoryGrid().get(x, y);
        TWAction action = TWAction.MOVE;
        if (temp instanceof TWTile && this.carriedTiles.size() < 3) {
            action = TWAction.PICKUP;
        } else if (temp instanceof TWHole && this.carriedTiles.size() > 0) {
            action = TWAction.PUTDOWN;
        }
        if (flag == 0) { //向自己的领域移动
            if (y+Parameters.defaultSensorRange < this.initalPosition[0][0]) {
                newDir = TWDirection.S;
            } else if (y-Parameters.defaultSensorRange > this.initalPosition[0][0]) {
                newDir = TWDirection.N;
            } else {
                if(t<0.5){newDir = TWDirection.W;}
                else{newDir = TWDirection.E;}
                flag = 1;
            }
        } else if (flag == 1) { //遍历自己的领域
            if (lastThought.getDirection() == TWDirection.S) { // 上一步是向下
                if (DownStep == 6 || y + Parameters.defaultSensorRange >= stopY) { //如果已经向下7步或者到达了最低
                    if (x - Parameters.defaultSensorRange <= 0) {  //判断是都在左边界
                        newDir = TWDirection.E;
                    } else if (x + Parameters.defaultSensorRange >= Parameters.xDimension - 1) { //判断是否在右边界
                        newDir = TWDirection.W;
                    } else {
                        newDir = TWDirection.E;
                    }
                    DownStep = 0;
                } else if (DStep == 1) {
                    newDir = TWDirection.E;
                    DStep = 0;
                } else if (DownStep < 6) {  //向下不足7步，继续向下
                    Object n = this.memory.getMemoryGrid().get(x, y + 1);
                    if (n instanceof TWObstacle && ((TWObstacle) n).getTimeLeft(time) > 3) {
                        newDir = TWDirection.W;
                        LeftStep++;
                    } else {
                        newDir = TWDirection.S;
                        DownStep++;
                    }
                }
            } else if (lastThought.getDirection() == TWDirection.E) { //上一步是向右
                if (RightStep == 1) {
                    newDir = TWDirection.N;
                    RightStep = 0;
                } else if (x + Parameters.defaultSensorRange >= Parameters.xDimension - 1) { //判断是到右边界
                    if (y + Parameters.defaultSensorRange >= stopY) { //判断是到下边界
                        newDir = TWDirection.N;
                    } else {
                        newDir = TWDirection.S;
                        DownStep++;
                    }
                } else { //没到右边界，继续向右
                    Object n = this.memory.getMemoryGrid().get(x + 1, y);
                    if (n instanceof TWObstacle && ((TWObstacle) n).getTimeLeft(time) > 3) {
                        newDir = TWDirection.S;
                        DStep++;

                    } else {
                        newDir = TWDirection.E;
                    }
                }
            } else if (lastThought.getDirection() == TWDirection.W) { //上一步是向左
                if (LeftStep == 1) {
                    newDir = TWDirection.S;
                    LeftStep = 0;
                } else if (x - Parameters.defaultSensorRange <= 0) { //判断是到左边界
                    if (y + Parameters.defaultSensorRange >= stopY) { //判断是到下边界
                        newDir = TWDirection.N;
                    } else {
                        newDir = TWDirection.S;
                        DownStep++;
                    }
                } else { //没左到边界，继续向左
                    Object n = this.memory.getMemoryGrid().get(x - 1, y);
                    if (n instanceof TWObstacle && ((TWObstacle) n).getTimeLeft(time) > 3) {
                        newDir = TWDirection.N;
                        UStep++;
                    } else {
                        newDir = TWDirection.W;
                    }
                }
            } else { //上一步是向上
                if (UpStep == 6 || y - Parameters.defaultSensorRange <= startY) { //如果已经向下7步或者到达了最低
                    if (x - Parameters.defaultSensorRange <= 0) {  //判断是都在左边界
                        newDir = TWDirection.E;
                    } else if (x + Parameters.defaultSensorRange >= Parameters.xDimension - 1) { //判断是否在右边界
                        newDir = TWDirection.W;
                    } else {
                        newDir = TWDirection.W;
                    }
                    UpStep = 0;
                } else if (UStep == 1) {
                    newDir = TWDirection.W;
                    UStep = 0;
                } else if (UpStep < 7) {  //向下不足7步，继续向下
                    Object n = this.memory.getMemoryGrid().get(x, y - 1);
                    if (n instanceof TWObstacle && ((TWObstacle) n).getTimeLeft(time) > 3) {
                        newDir = TWDirection.E;
                        RightStep++;
                    } else {
                        newDir = TWDirection.N;
                        UpStep++;
                    }
                }
            }
        }
        TWThought tt = new TWThought(action, newDir);
        if (action == TWAction.PICKUP) {
            tt.setTile((TWTile) temp);
        } else if (action == TWAction.PUTDOWN) {
            tt.setHole((TWHole) temp);
        }
        return tt;
    }


//    private TWThought getExploreThought() {
//
////        if(x-Parameters.defaultSensorRange<=0 && y-Parameters.defaultSensorRange <=0){
////            return new TWThought(TWAction.MOVE, TWDirection.E);
////        }
////        if(x-Parameters.defaultSensorRange<=0 && y+Parameters.defaultSensorRange >= Parameters.yDimension){
////            return new TWThought(TWAction.MOVE, TWDirection.N);
////        }
////        if(x+Parameters.defaultSensorRange>=Parameters.xDimension && y-Parameters.defaultSensorRange <=0){
////            return new TWThought(TWAction.MOVE, TWDirection.S);
////        }
////        if(x+Parameters.defaultSensorRange>=Parameters.xDimension && y+Parameters.defaultSensorRange >= Parameters.yDimension){
////            return new TWThought(TWAction.MOVE, TWDirection.W);
////        }
////        return new TWThought(TWAction.MOVE, this.getRandomDirection());
//        java.util.List<TWDirection> iMoves = getIndividualMaxExploreDirection(x, y);
//        //        ArrayList<TWAgent> otheragent = this.otherAgents;
//        if (this.otherAgents != null) {
//            java.util.List<TWDirection> gMoves = new ArrayList<>(Arrays.asList(TWDirection.W, TWDirection.E, TWDirection.N, TWDirection.S));
//            for (int i = 0; i < this.otherAgents.size(); i++) {
//                gMoves = getGroupMaxExploreDirection(x, y, this.otherAgents.get(i).getX(), this.otherAgents.get(i).getY(), gMoves);
////                if (gMoves.contains(TWDirection.W) && gMoves.contains(TWDirection.N)) {
////                    double t = Math.random();
////                    if (t < 0.5) {
////                        return new TWThought(TWAction.MOVE, TWDirection.W);
////                    } else {
////                        return new TWThought(TWAction.MOVE, TWDirection.N);
////                    }
////                }
////                if (iMoves.contains(TWDirection.W) && iMoves.contains(TWDirection.N)) {
////                    double t = Math.random();
////                    if (t < 0.5) {
////                        return new TWThought(TWAction.MOVE, TWDirection.W);
////                    } else {
////                        return new TWThought(TWAction.MOVE, TWDirection.N);
////                    }
////                }
//            }
//            iMoves.retainAll(gMoves);
//            if (iMoves.size()>0) {
//                int rand = (int) (Math.random() * iMoves.size());
//                return new TWThought(TWAction.MOVE, iMoves.get(rand));
//            }else {
//
//                int rand = (int) (Math.random() * gMoves.size());
//                return new TWThought(TWAction.MOVE, gMoves.get(rand));
//            }
//        }
//        else {
//            if (iMoves.contains(TWDirection.W) && iMoves.contains(TWDirection.N)) {
//                double t = Math.random();
//                if (t < 0.5) {
//                    return new TWThought(TWAction.MOVE, TWDirection.W);
//                } else {
//                    return new TWThought(TWAction.MOVE, TWDirection.N);
//                }
//            }
//            int rand = (int) (Math.random() * iMoves.size());
//            return new TWThought(TWAction.MOVE, iMoves.get(rand));
//        }
//    }

//    private double fuelControlUrgency(int x, int y, double fuelLevel){
//        if (fuelLevel >= this.getFuelDistance() + 20){
//            return 1;
//        }else{
//            double t = ()
//        }
//    }

//    private TWAgent getTheOtherAgent(){
//        for(TWAgent agent : this.getEnvironment().getAgents()){
//            if(agent!=this){ return agent; }
//        }
//        return null;
//    }


    private  int getFuelDistance() {
        if (this.fuelStation == null) {
            return this.getEnvironment().getxDimension() + this.getEnvironment().getyDimension();
        }
        return Math.abs(this.getX() - this.fuelStation.getX()) + Math.abs(this.getY() - this.fuelStation.getY());
    }

    private void checkFuelStation(){
        if(this.fuelStation==null|| (this.fuelx==-1|| this.fuely==-1)){
            ObjectGrid2D memory=this.getMemory().getMemoryGrid();
            for(int i=0;i<this.MapSizeX;i++){
                for(int j=0;j<this.MapSizeY;j++){
                    if(memory.get(i,j) instanceof TWFuelStation){
                        this.fuelStation=(TWFuelStation)memory.get(i,j);
                        this.fuelx=i;
                        this.fuely=j;
                        System.out.println(this.name+" find the fuelStation!");
                        return;
                    }
                }
            }
        }
        else {
            System.out.println(this.name + " find the fuelStation!");
        }
    }
    private TWThought getFuelThought(TWThought lastThought) {
//        if (this.fuelStation == null|| (this.fuelx==-1|| this.fuely==-1)) {
//            TWFuelStation tile = this.getMemory().getNearbyFuelStation(this.getX(), this.getY());
//            if (tile == null) {
//                for (int i = 0; i < this.otherAgents.size(); i++) {
//                    tile = this.otherAgents.get(i).getMemory().getNearbyFuelStation(
//                            this.otherAgents.get(i).getX(), this.otherAgents.get(i).getY());
//                    if (tile != null) break;
//                }
//            }
//            if (tile == null) return this.getExploreThought();
//            this.fuelStation = tile;
//            this.fuelStation = this.getEnvironment().getFuelingStation();

//            Object tempClosestItem=this.getClosest(TWFuelStation.class);
//            if(tempClosestItem!=null){
//                this.fuelStation= (TWFuelStation) tempClosestItem;
//                this.fuelx=this.fuelStation.getX();
//                this.fuely=this.fuelStation.getY();
//            }
//            else {
//                this.fuelStation = null;
//            }
//        }
        if (this.fuelStation == null|| (this.fuelx==-1|| this.fuely==-1)){
            System.out.println(this.name+" try to explore to find fuel");
            return this.getExploreThought(lastThought);
        }
//        TWDirection d = this.getOneStepDirection(
//                this.fuelStation.getX(), this.fuelStation.getY());
        TWDirection d=this.getOneStepDirectionToFuelStation();
        if (d == TWDirection.Z) {
            this.state = State.GET_TILE;
            return new TWThought(TWAction.REFUEL, d);
        } else {
            return new TWThought(TWAction.MOVE, d);
        }
    }

    private TWThought getHoleThought(TWThought lastThought) {
//        if (this.targetHole == null) {
//            this.targetHole = this.getMemory().getNearbyHole(this.getX(), this.getY(), 90);
            Object tempClosestItem=this.getClosest(TWHole.class);
            Object tempClosestItem_second=this.getClosest(TWTile.class);
            if(tempClosestItem!=null){
                this.targetHole= (TWHole) tempClosestItem;
            }
            else {
                this.targetHole = null;
            }
//        }
        if ((tempClosestItem_second!=null)&& this.carriedTiles.size()<3){
            if(this.targetHole!=null && this.distances[((TWTile)tempClosestItem_second).getX()][((TWTile)tempClosestItem_second).getY()]<
                    this.distances[this.targetHole.getX()][this.targetHole.getY()]){
                if(this.distances[((TWTile)tempClosestItem_second).getX()][((TWTile)tempClosestItem_second).getY()]<this.senseRange){
                    return this.getTileThought(lastThought);
                }
            }
            else if(this.targetHole==null){
                return this.getTileThought(lastThought);
            }
        }
        if (this.targetHole == null) return this.getExploreThought(lastThought);

        TWDirection d = this.getOneStepDirection(this.targetHole.getX(), this.targetHole.getY());
        if (d == TWDirection.Z) {
            TWThought t = new TWThought(TWAction.PUTDOWN, d);
            t.setHole(this.targetHole);
            this.targetHole = null;
            return t;
        } else {
            return new TWThought(TWAction.MOVE, d);
        }
    }

    private  TWThought getTileThought(TWThought lastThought) {
//        if (this.targetTile == null) {
//            this.targetTile = this.getMemory().getNearbyTile(this.getX(), this.getY(), 100);
            Object tempClosestItem=this.getClosest(TWTile.class);
            Object tempClosestItem_second=this.getClosest(TWHole.class);
            if(tempClosestItem!=null){
                this.targetTile= (TWTile) tempClosestItem;
//                System.out.println("this tile have time: " +(this.targetTile.getTimeLeft(this.getMemory().schedule.getTime())+" the real location "+
//                        this.targetTile.getX()+" "+this.targetTile.getY()+
//                        " was record at "+this.getX()+" "+this.getY()));
            }
            else {
                this.targetTile = null;
            }
//        }
        if ((tempClosestItem_second!=null)&& this.carriedTiles.size()>0){
            if(this.targetTile!=null && this.distances[((TWHole)tempClosestItem_second).getX()][((TWHole)tempClosestItem_second).getY()]<
                    this.distances[this.targetTile.getX()][this.targetTile.getY()]){
                if(this.distances[((TWHole)tempClosestItem_second).getX()][((TWHole)tempClosestItem_second).getY()]<this.senseRange){
                    return this.getHoleThought(lastThought);
                }
            }
            else if(this.targetTile==null){
                return this.getHoleThought(lastThought);
            }
        }
        if (this.targetTile == null) return this.getExploreThought(lastThought);
        TWDirection d = this.getOneStepDirection(this.targetTile.getX(), this.targetTile.getY());
        if (d == TWDirection.Z) {
            if(!(this.getMemory().getMemoryGrid().get(this.getX(),this.getY()) instanceof TWTile))
            {
                System.out.println("the target tile disappear");
            }
//            System.out.println("this tile have time: " +((TWObject)this.getMemory().getMemoryGrid().get(this.getX(),this.getY())).getTimeLeft(this.getMemory().schedule.getTime())+" the real location "+
//                    ((TWObject)this.getMemory().getMemoryGrid().get(this.getX(),this.getY())).getX()+" "+((TWObject)this.getMemory().getMemoryGrid().get(this.getX(),this.getY())).getY()+
//                    " was record at "+this.getX()+" "+this.getY());
            TWThought t = new TWThought(TWAction.PICKUP, d);
            t.setTile(this.targetTile);
            this.targetTile = null;
            return t;
        } else {
            return new TWThought(TWAction.MOVE, d);
        }
    }
    protected TWDirection getRandomDirection(){

        TWDirection randomDir = TWDirection.values()[this.getEnvironment().random.nextInt(5)];

        if(this.getX()>=this.getEnvironment().getxDimension() ){
            randomDir = TWDirection.W;
        }else if(this.getX()<=1 ){
            randomDir = TWDirection.E;
        }else if(this.getY()<=1 ){
            randomDir = TWDirection.S;
        }else if(this.getY()>=this.getEnvironment().getxDimension() ){
            randomDir = TWDirection.N;
        }

        return randomDir;

    }
}
