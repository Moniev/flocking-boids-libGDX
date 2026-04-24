package com.moniev.boids.core.Obstacle;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.moniev.boids.core.Vector.Vector;

public class Obstacle {
    public Vector position;
    public float radius;
    public ModelInstance modelInstance;

    public Obstacle(Vector position, float radius, Model model) {
        this.position = position;
        this.radius = radius;
        this.modelInstance = new ModelInstance(model);
        this.modelInstance.transform.setToTranslation(position.x, position.y, position.z);
        this.modelInstance.transform.scl(radius * 2);
    }
}
