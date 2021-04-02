/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package tileworld.agent;
import sim.util.Int2D;

import tileworld.environment.*;
import tileworld.exceptions.CellBlockedException;

import java.util.ArrayList;
import java.util.List;
import tileworld.Parameters;

enum State {
    GET_FUEL,
    GET_TILE,
    GET_HOLE,
    EXPLORE
}

/**
 * SimpleStateTWAgent
 *
 * @author changqing.zhou
 */
public class SimpleStateTWAgent extends SimpleTWAgent{
    private ArrayList<TWAgent> otherAgents;
    private State state;
    private TWFuelStation fuelStation;
    private TWTile targetTile;
    private TWHole targetHole;
    private TWAgentPercept[][] objects;

    public SimpleStateTWAgent(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(name, xpos, ypos, env, fuelLevel);
        this.otherAgents = new ArrayList<TWAgent>();
        this.state = State.GET_FUEL;
        this.fuelStation = null;
        this.targetHole = null;
        this.targetTile = null;

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

    protected TWThought think() {
//        getMemory().getClosestObjectInSensorRange(Tile.class);
        System.out.println("Simple Score: " + this.score);
        // return new TWThought(TWAction.MOVE, getRandomDirection());
        TWThought thought;
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
                return TWDirection.Z;
            }
        }
    }

    private TWThought getExploreThought() {
//        return new TWThought(TWAction.MOVE, this.getRandomDirection());
        List<TWDirection> iMoves = getIndividualMaxExploreDirection(x, y);
        //        ArrayList<TWAgent> otheragent = this.otherAgents;
        if (this.otherAgents != null) {
            for (int i = 0; i < this.otherAgents.size(); i++) {
                List<TWDirection> gMoves = getGroupMaxExploreDirection(x, y, this.otherAgents.get(i).getX(), this.otherAgents.get(i).getY(), iMoves);
                if (gMoves.contains(TWDirection.W) && gMoves.contains(TWDirection.N)) {
                    double t = Math.random();
                    if (t < 0.5) {
                        return new TWThought(TWAction.MOVE, TWDirection.W);
                    } else {
                        return new TWThought(TWAction.MOVE, TWDirection.N);
                    }
                }
                if (iMoves.contains(TWDirection.W) && iMoves.contains(TWDirection.N)) {
                    double t = Math.random();
                    if (t < 0.5) {
                        return new TWThought(TWAction.MOVE, TWDirection.W);
                    } else {
                        return new TWThought(TWAction.MOVE, TWDirection.N);
                    }
                }

                int rand = (int) (Math.random() * gMoves.size());
                return new TWThought(TWAction.MOVE, gMoves.get(rand));
            }
        }
        else {
            if (iMoves.contains(TWDirection.W) && iMoves.contains(TWDirection.N)) {
                double t = Math.random();
                if (t < 0.5) {
                    return new TWThought(TWAction.MOVE, TWDirection.W);
                } else {
                    return new TWThought(TWAction.MOVE, TWDirection.N);
                }
            }
            int rand = (int) (Math.random() * iMoves.size());
            return new TWThought(TWAction.MOVE, iMoves.get(rand));
        }
        int rand = (int) (Math.random() * iMoves.size());
        return new TWThought(TWAction.MOVE, iMoves.get(rand));
    }

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

    private List<TWDirection> getIndividualMaxExploreDirection(int curX, int curY){
        List<TWDirection> moves = new ArrayList();

        int maxExplored = 0;
        int explored;
        if(this.getEnvironment().isInBounds(curX+1, curY)){
            explored = this.getMemory().countNewlyExplored(curX+1, curY, curX, curY, memory.getSimulationTime());
            if(explored>maxExplored){
                maxExplored = explored;
                moves = new ArrayList();
                moves.add(TWDirection.E);
            } else if(explored==maxExplored){
                moves.add(TWDirection.E);
            }
        }
        if(this.getEnvironment().isInBounds(curX-1, curY)){
            explored = this.getMemory().countNewlyExplored(curX-1, curY, curX, curY, memory.getSimulationTime());
            if(explored>maxExplored){
                maxExplored = explored;
                moves = new ArrayList();
                moves.add(TWDirection.W);
            } else if(explored==maxExplored){
                moves.add(TWDirection.W);
            }
        }
        if(this.getEnvironment().isInBounds(curX, curY+1)){
            explored = this.getMemory().countNewlyExplored(curX, curY+1, curX, curY, memory.getSimulationTime());
            if(explored>maxExplored){
                maxExplored = explored;
                moves = new ArrayList();
                moves.add(TWDirection.S);
            } else if(explored==maxExplored){
                moves.add(TWDirection.S);
            }
        }
        if(this.getEnvironment().isInBounds(curX, curY-1)){
            explored = this.getMemory().countNewlyExplored(curX, curY-1, curX, curY, memory.getSimulationTime());
            if(explored>maxExplored){
                maxExplored = explored;
                moves = new ArrayList();
                moves.add(TWDirection.N);
            } else if(explored==maxExplored){
                moves.add(TWDirection.N);
            }
        }
        return moves;
    }


    private List<TWDirection> getGroupMaxExploreDirection(int curX, int curY, int agentX, int agentY, List<TWDirection> directions){
        List<TWDirection> result = new ArrayList();
        for(TWDirection direction : directions){

            Int2D nextPos = direction.advance(new Int2D(curX, curY));

            int newDist = this.getMemory().distance(nextPos.x, nextPos.y, agentX, agentY);
            int oldDist = this.getMemory().distance(curX, curY, agentX, agentY);

            if(newDist < oldDist && (Math.abs(nextPos.x - agentX)>2*Parameters.defaultSensorRange+1 || Math.abs(nextPos.y - agentY)>2*Parameters.defaultSensorRange+1)){
                result.add(direction);
            }
        }
        if(result.isEmpty()){
            return directions;
        } else{
            return result;
        }
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
        if (this.targetHole == null) {
            this.targetHole = this.getMemory().getNearbyHole(this.getX(), this.getY(), 90);
        }
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
        if (this.targetTile == null) {
            this.targetTile = this.getMemory().getNearbyTile(this.getX(), this.getY(), 100);
        }
        if (this.targetTile == null) return this.getExploreThought();
        TWDirection d = this.getOneStepDirection(this.targetTile.getX(), this.targetTile.getY());
        if (d == TWDirection.Z) {
            TWThought t = new TWThought(TWAction.PICKUP, d);
            t.setTile(this.targetTile);
            this.targetTile = null;
            return t;
        } else {
            return new TWThought(TWAction.MOVE, d);
        }
    }
}
