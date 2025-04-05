package com.moniev.boids.core.MainEngine;

import java.util.concurrent.ThreadLocalRandom;

import com.badlogic.gdx.graphics.VertexAttributes.Usage;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.moniev.boids.core.Boid.Boid;
import com.moniev.boids.core.Vector.Vector;


public class Engine {
    public final ModelBuilder modelBuilder;
    public final int boidsLimit;
    public final int size;
    public final int subSteps;
    public final float subStepDt;
    public float mTime;
    public float mFrameDt;
    public Octree tree;

    public Engine(int boidsLimit, int size, int subSteps, float rate) {
        Vector center = new Vector(0, 0, 0);
        this.subSteps = subSteps;
        this.modelBuilder = new ModelBuilder();
        this.mFrameDt = 1.f / rate;
        this.tree = new Octree(center, size, 16, modelBuilder, mFrameDt, this);
        this.boidsLimit = boidsLimit;
        this.size = size;
        this.subStepDt = mFrameDt / (float)subSteps;
    }

    public float randomFloat(float min, float max) {
        return min + (max - min) * ThreadLocalRandom.current().nextFloat();
    }

    public Vector randomVector(float min, float max) {
        return new Vector(
            randomFloat(min, max),
            randomFloat(min, max),
            randomFloat(min, max)
        ); 
    }

    public void addBoid(int i) {
        Model boidModel = modelBuilder.createCone(
            1f, 3f, 1f, 3,
            calculateColor(),
            Usage.Position | Usage.Normal
        );
        ModelInstance modelInstance = new ModelInstance(boidModel);
        
        Vector position = randomVector(-16, 16);
        Boid boid = new Boid(position, boidModel, modelInstance, 60f, 60f);
        Vector velocity = new Vector(1f, 1f, 1f); 
        boid.setVelocity(velocity, mFrameDt);
        tree.addBoid(boid);
    }

    public Vector calculateCoordinates(int i) {
        float turnFraction = (float) ((3 - Math.sqrt(5)) * Math.PI);
        float normIter = (float) i / (boidsLimit - 1);
        float scale = 10f; 
        float distance = scale * (float) Math.sqrt(normIter); 
        float angle = 2 * (float) Math.PI * turnFraction * i;

        float x = distance * (float) Math.cos(angle);
        float y = distance * (float) Math.sin(angle);
        float z = scale * (float) Math.sin(normIter * Math.PI);

        return new Vector(x, y, z);
    }

    public Material calculateColor() {
        float r = (float)Math.sin((double)mTime / 4);
        float g = (float)Math.sin((double)mTime / 4 + 0.33f * 2.0f * Math.PI);
        float b = (float)Math.sin((double)mTime / 4 + 0.66f * 2.0f * Math.PI);
        float a = 0f;
        return new Material(ColorAttribute.createDiffuse(r, g, b, a));
    }

    public void renderBoids(ModelBatch modelBatch) {
        tree.renderBoids(modelBatch, tree.root);
    }

    public void renderTree(ModelBatch modelBatch) {
        tree.renderTree(modelBatch, tree.root);
    }
    
    public void update() {
        mTime += mFrameDt;
        for(int i = 0; i < subSteps; i++) {
            tree.resolveInnerAdjustment(tree.root);
            // tree.resolveOuterAdjustment(tree.root);
            tree.updateSpatialLookup(tree.root);
            tree.updateBoids(tree.root, subStepDt);
        }
    }

    public void disposeBoids() {
        tree.disposeBoids(tree.root);
    }

    public void disposeTree(){
        tree.disposeTree(tree.root);
    }
}
