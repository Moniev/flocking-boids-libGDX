package com.moniev.boids.core.Boid;


import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.moniev.boids.core.Vector.Vector;


public class Boid {
    public Vector position, lastPosition, acceleration; 
    public Model model;
    public ModelInstance modelInstance;
    
    public Boid(Vector position, Model model, ModelInstance modelInstance) {
        this.position = new Vector(position);
        this.lastPosition = new Vector(position);
        this.model = model;
        this.modelInstance = modelInstance;
        this.acceleration = new Vector(0, 0, 0);
    }

    public void accelerate(Vector a) {
        acceleration.set(acceleration.add(a)); 
    }

    public void setVelocity(Vector v, float dt) {
        lastPosition.set(position.substract(v.multiply(dt))); 
    }

    public void addVelocity(Vector v, float dt) {
        lastPosition.set(lastPosition.substract(v.multiply(dt))); 
    }   

    public Vector getVelocity(float dt) { 
        return position.substract(lastPosition).subdivide(dt); 
    } 

    public void printBoid() {
        String sPosition = position.toString();
        String sLastPosition = lastPosition.toString();
        System.out.printf("[position: %s][last position %s]\n", sPosition, sLastPosition);
    }

    public void update(float dt) {        
        Vector displacement = position.substract(lastPosition);        
        Vector newPosition = position.add(displacement).add(acceleration.multiply(dt * dt));
    
        lastPosition.set(position); 
        position.set(newPosition); 
    }
}


