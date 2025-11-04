package com.moniev.boids.core.Boid;

import java.util.ArrayList;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.moniev.boids.core.Vector.Vector;

public class Boid {
  public Vector position, lastPosition, acceleration;
  public final Model model;
  public ModelInstance modelInstance;
  private final float minVelocity, maxVelocity;

  private static final ArrayList<Vector> directions = new ArrayList<>();
  private static final int directionsLimit = 120;

  static {
    final float angle = 2.39996322972865332f;
    for (int i = 0; i < directionsLimit; i++) {
      float y = 1.0f - ((i + 0.5f) / directionsLimit) * 2.0f;
      float radiusAtY = (float) Math.sqrt(1.0f - y * y);
      float theta = angle * i;

      float x = (float) Math.cos(theta) * radiusAtY;
      float z = (float) Math.sin(theta) * radiusAtY;

      directions.add(new Vector(x, y, z));
    }
  }

  public Boid(Vector position, Vector initialVelocity, float dt, Model model, ModelInstance modelInstance,
      float minVelocity, float maxVelocity) {
    this.position = new Vector(position);
    this.lastPosition = position.substract(initialVelocity.multiply(dt));
    this.model = model;
    this.modelInstance = modelInstance;
    this.acceleration = new Vector(0, 0, 0);
    this.minVelocity = minVelocity;
    this.maxVelocity = maxVelocity;
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
    Vector velocity = getVelocity(dt);
    float speed = velocity.length();

    Vector limitedVelocity = velocity;
    if (speed > maxVelocity) {
      limitedVelocity = velocity.subdivide(speed).multiply(maxVelocity);
    } else if (speed < minVelocity) {
      if (speed != 0) {
        limitedVelocity = velocity.subdivide(speed).multiply(minVelocity);
      }
    }

    setVelocity(limitedVelocity, dt);

    Vector displacement = position.substract(lastPosition);
    Vector newPosition = position.add(displacement).add(acceleration.multiply(dt * dt));

    lastPosition.set(position);
    position.set(newPosition);

    acceleration.set(0);
  }

  public ArrayList<Vector> getFieldOfView(float axis, float dt) {
    ArrayList<Vector> forwardDirections = new ArrayList<>();
    Vector velocity = getVelocity(dt);

    if (velocity.length() == 0) {
      return forwardDirections;
    }

    Vector forward = velocity.normalize();

    float maxAngle = (float) Math.toRadians(axis / 2f);
    float minDotProduct = (float) Math.cos(maxAngle);

    for (Vector direction : directions) {
      float dotProduct = forward.dotProduct(direction);

      if (dotProduct > minDotProduct) {
        forwardDirections.add(direction);
      }
    }

    return forwardDirections;
  }
}
