package tileworld.agent;

import tileworld.Parameters;
import tileworld.environment.*;

import java.awt.*;
import java.util.ArrayList;
import java.util.Map.Entry;
import java.util.Arrays;

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
    protected int fuelx;
    protected int fuely;
    protected double fuelTolerance;
    protected int MapSizeX;
    protected int MapSizeY;
    //~~~~~~~~~~~naive~~~~~~
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
    public TWAgent1(String name,int xpos, int ypos, TWEnvironment env, double fuelLevel) {
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
        ArrayList<int []> openset=new ArrayList<int []>();
        ArrayList<int []> closeset=new ArrayList<int []>();
        openset.add(new int[] {this.getX(),this.getY()});
        g_score= new int[this.MapSizeX][this.MapSizeY];;
        h_score=new int[this.MapSizeX][this.MapSizeY];;
        f_score=new int[this.MapSizeX][this.MapSizeY];;
        came_from=new int[this.MapSizeX][this.MapSizeY][2];
        for(int i=0;i<g_score.length;i++){
            Arrays.fill(g_score[i],MapSizeX+MapSizeY);
            Arrays.fill(h_score[i],MapSizeX+MapSizeY);
            Arrays.fill(f_score[i],MapSizeX+MapSizeY);
        }

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
            int min_distance=MapSizeX+MapSizeY;
            for(int i=-1;i<2;i++){
                for(int j=-1;j<2;j++){
                    if(i*j!=0||(i==0 && j==0)){
                        continue;
                    }
                    int better_estimate_g=0;
                    if(this.getEnvironment().isInBounds(candidate[0]+i,candidate[1]+j) &&
                            !(this.memory.getMemoryGrid().get(candidate[0]+i,candidate[1]+j) instanceof TWObstacle)){
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
        while(ifArrayContain(distances,-1)){
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
            }
            this.calculatedArea.addAll(this.toCalculateArea);
            this.toCalculateArea=new ArrayList<int[]>();
            int ifchange1=0;
            for (int i=1;i<this.MapSizeX;i++){
                for(int j=1;j<this.MapSizeY;j++){
                    if(tempDistances[i][j]!=distances[i][j]){
                        ifchange1=1;
                        break;
                    }
                }
                if(ifchange1==1){
                    break;
                }
            }
            if(ifchange1!=1){
                break;
            }
        }
    }
    public Boolean checkCheckable(int aimx, int aimy){
        if(this.getEnvironment().isInBounds(aimx,aimy)
                && (!(this.memory.getMemoryGrid().get(aimx,aimy) instanceof TWObstacle)
                || (aimx==this.getX() && aimy==this.getY()))){
            return true;
        }
        else{
            return false;
        }
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
    public int[] get_min(ArrayList<int[]> array,int[][] scores){
        int min=MapSizeX+MapSizeY;
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

    protected Object getClosest(Class Type){
        int minDistance=this.MapSizeX+this.MapSizeY;
        int minx=-1;
        int miny=-1;
//        this.displayMap(this.distances);
        for(int i=0;i<this.MapSizeX;i++){
            for(int j=0; j<this.MapSizeY;j++){
                if(this.distances[i][j]!=-1&&this.distances[i][j]<minDistance && Type.isInstance( this.memory.getMemoryGrid().get(i,j))){
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
    protected TWThought think(){
//        this.sensor.sense();
        //state 0: exploration
        //state 1: ToTile
        //state 2: ToHole
        //state 3: ToFuel
        System.out.println(" Simple Score: " + this.score+" name "+this.name+ " state: "+this.state+" feul level "+this.fuelLevel+"~~~~~~~~~~~~~~~~~~~~~~~~~~~");
        // return new TWThought(TWAction.MOVE, getRandomDirection());
        TWThought thought;

        this.waterFlood(this.getX(),this.getY());
//        displayMap(this.distances);
        switch (this.state) {
            case EXPLORE:
                thought = this.getExploreThought();
                this.state = State.GET_TILE;
                break;
            case GET_FUEL:
                thought = this.getFuelThought();
                if (thought.getAction() == TWAction.REFUEL) {
                    this.state = State.GET_TILE;
                }
                break;
            case GET_HOLE:
                thought = this.getHoleThought();
                if (thought.getAction() == TWAction.PUTDOWN) {
                    this.state = State.GET_TILE;
                }
                break;
            case GET_TILE:
                thought = this.getTileThought();
                if (thought.getAction() == TWAction.PICKUP) {
                    this.state = State.GET_HOLE;
                }
                break;
            default:
                thought = this.getExploreThought();
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

    private TWThought getExploreThought() {
        return new TWThought(TWAction.MOVE, this.getRandomDirection());
    }

    private  int getFuelDistance() {
        if (this.fuelStation == null) {
            return this.getEnvironment().getxDimension() + this.getEnvironment().getyDimension();
        }
        return Math.abs(this.getX() - this.fuelStation.getX()) + Math.abs(this.getY() - this.fuelStation.getY());
    }

    private TWThought getFuelThought() {
        if (this.fuelStation == null) {
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
            this.fuelStation = this.getEnvironment().getFuelingStation();
        }
        TWDirection d = this.getOneStepDirection(
                this.fuelStation.getX(), this.fuelStation.getY());
        if (d == TWDirection.Z) {
            this.state = State.GET_TILE;
            return new TWThought(TWAction.REFUEL, d);
        } else {
            return new TWThought(TWAction.MOVE, d);
        }
    }

    private TWThought getHoleThought() {
//        if (this.targetHole == null) {
//            this.targetHole = this.getMemory().getNearbyHole(this.getX(), this.getY(), 90);
            Object tempClosestItem=this.getClosest(TWHole.class);
            if(tempClosestItem!=null){
                this.targetHole= (TWHole) tempClosestItem;
            }
            else {
                this.targetHole = null;
            }
//        }
        if (this.targetHole == null) return this.getExploreThought();

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

    private  TWThought getTileThought() {
//        if (this.targetTile == null) {
//            this.targetTile = this.getMemory().getNearbyTile(this.getX(), this.getY(), 100);
            Object tempClosestItem=this.getClosest(TWTile.class);
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
        if (this.targetTile == null) return this.getExploreThought();
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
