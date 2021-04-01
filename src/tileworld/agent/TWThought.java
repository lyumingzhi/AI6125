/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package tileworld.agent;

import tileworld.environment.TWDirection;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;

/**
 * TWContextBuilder
 *
 * @author michaellees
 * Created: Jan 24, 2011
 *
 * Copyright 2011
 *
 *
 * Description:
 *
 * This small class is used to return the result of a think execution.
 *
 * The result of a think procedure should include two things: the actions to take
 * and a direction in which to move if a move is selected.
 *
 */
public class TWThought {

    private final TWAction action;
    private final TWDirection direction;
    private TWTile tile;
    private TWHole hole;

    public TWThought(TWAction action, TWDirection direction) {
        this.direction = direction;
        this.action = action;
        this.tile = null;
        this.hole = null;
    }

    public TWAction getAction() {
        return action;
    }

    public TWDirection getDirection() {
        return direction;
    }

    public TWTile getTile() {
        return tile;
    }

    public void setTile(TWTile tile) {
        this.tile = tile;
    }

    public TWHole getHole() {
        return hole;
    }

    public void setHole(TWHole hole) {
        this.hole = hole;
    }


}
