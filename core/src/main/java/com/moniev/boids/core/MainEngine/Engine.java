package com.moniev.boids.core.MainEngine;

import java.util.Random;
import java.util.ArrayList;
import java.util.concurrent.ThreadLocalRandom;
import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.VertexAttributes.Usage;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.moniev.boids.core.Boid.Boid;
import com.moniev.boids.core.Vector.Vector;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.graphics.g3d.utils.MeshPartBuilder;
import com.badlogic.gdx.graphics.GL20;

public class Engine {
  public final ModelBuilder modelBuilder;
  public final int boidsNumber;
  public final int boidsLimit;
  public final int size;
  public final int subSteps;
  public final float subStepDt;
  public float mTime;
  public float mFrameDt;
  public Octree tree;
  public ArrayList<Boid> boids;
  private final Model sharedBoidModel;
  private final Model sharedNodeWireframeModel;

  public Engine(int boidsNumber, int boidsLimit, int size, int subSteps, float rate) {
    Vector center = new Vector(0, 0, 0);
    this.subSteps = subSteps;
    this.modelBuilder = new ModelBuilder();
    this.mFrameDt = 1.f / rate;
    this.boidsLimit = boidsLimit;
    this.boidsNumber = boidsNumber;
    this.size = size;
    this.subStepDt = mFrameDt / (float) subSteps;
    this.boids = new ArrayList<Boid>();
    this.sharedBoidModel = modelBuilder.createCone(
        1f, 3f, 1f, 3,
        new Material(ColorAttribute.createDiffuse(1, 1, 1, 1)),
        Usage.Position | Usage.Normal);
    this.sharedNodeWireframeModel = createWireframeCube(modelBuilder);
    this.tree = new Octree(center, size, boidsLimit, 16, modelBuilder, mFrameDt, this);
  }

  public void initializeBoids(int count) {
    Random random = new Random();
    float startSpeed = 45.0f;

    for (int i = 0; i < count; i++) {
      ModelInstance modelInstance = new ModelInstance(sharedBoidModel);
      Vector position = randomVector(-size / 2f, size / 2f);

      Vector velocity = new Vector(
          random.nextFloat() * 2 - 1,
          random.nextFloat() * 2 - 1,
          random.nextFloat() * 2 - 1);
      velocity.normalize().multiply(startSpeed);

      Boid boid = new Boid(position, velocity, subStepDt, sharedBoidModel, modelInstance, 20f, 30f,
          30f);
      boids.add(boid);
    }
  }

  private Model createWireframeCube(ModelBuilder builder) {
    float size = 0.5f;

    builder.begin();
    MeshPartBuilder meshBuilder = builder.part("lines", GL20.GL_LINES,
        Usage.Position, new Material());
    meshBuilder.setColor(Color.GREEN);
    meshBuilder.box(size * 2, size * 2, size * 2);

    return builder.end();
  }

  public Model getNodeModel() {
    return sharedNodeWireframeModel;
  }

  public float randomFloat(float min, float max) {
    return min + (max - min) * ThreadLocalRandom.current().nextFloat();
  }

  public Vector randomVector(float min, float max) {
    return new Vector(
        randomFloat(min, max),
        randomFloat(min, max),
        randomFloat(min, max));
  }

  public void addBoid(int i) {
    ModelInstance modelInstance = new ModelInstance(sharedBoidModel);

    Vector position = randomVector(-16, 16);
    Vector velocity = new Vector(30, 0, 30);
    Boid boid = new Boid(position, velocity, subStepDt, sharedBoidModel, modelInstance, 30f, 60f,
        60f);
    boids.add(boid);
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

  public ColorAttribute calculateColor() {
    float r = (float) Math.sin((double) mTime / 4);
    float g = (float) Math.sin((double) mTime / 4 + 0.33f * 2.0f * Math.PI);
    float b = (float) Math.sin((double) mTime / 4 + 0.66f * 2.0f * Math.PI);
    float a = 1f;
    return ColorAttribute.createDiffuse(r, g, b, a);
  }

  public ColorAttribute green() {
    return ColorAttribute.createDiffuse(Color.FOREST);
  }

  public void renderTree(ModelBatch modelBatch) {
    tree.renderTree(modelBatch, tree.root);
  }

  public void update() {
    mTime += mFrameDt;

    Vector center = new Vector(0, 0, 0);
    this.tree = new Octree(center, size, boidsLimit, 16, modelBuilder, mFrameDt, this);

    for (Boid boid : boids) {
      tree.addBoid(boid);
    }

    for (int i = 0; i < subSteps; i++) {
      for (Boid boid : boids) {
        tree.calculateForces(boid);
      }

      for (Boid boid : boids) {
        boid.update(subStepDt);
      }
    }
  }

  public void renderBoids(ModelBatch modelBatch) {
    ColorAttribute color = green();
    for (Boid boid : boids) {
      Vector _target = new Vector(boid.position).add(boid.getVelocity(subStepDt));
      Vector3 target = new Vector3(_target.x, _target.y, _target.z).nor();

      Quaternion rotation = new Quaternion();
      rotation.setFromCross(Vector3.Y, target);

      boid.modelInstance.transform.set(rotation);
      boid.modelInstance.transform.setTranslation(
          boid.position.x, boid.position.y, boid.position.z);

      boid.modelInstance.materials.get(0).set(color);

      modelBatch.render(boid.modelInstance);
    }
  }

  public void disposeBoids() {
    sharedBoidModel.dispose();
    sharedNodeWireframeModel.dispose();
  }
}
