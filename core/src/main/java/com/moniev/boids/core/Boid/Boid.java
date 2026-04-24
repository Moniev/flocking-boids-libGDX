package com.moniev.boids.core.Boid;

import java.util.ArrayList;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.moniev.boids.core.Vector.Vector;
import com.moniev.boids.core.Obstacle.Obstacle;
import com.moniev.boids.core.Ray.Ray;

public class Boid {
  public Vector position, lastPosition, acceleration, currentAcceleration;
  public final Model model;
  public ModelInstance modelInstance;
  private final float minVelocity, maxVelocity;

  private static final ArrayList<Vector> directions = new ArrayList<>();
  private static final int directionsLimit = 120;

  private final float maxJerk;

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
      float minVelocity, float maxVelocity, float maxJerk) {
    this.position = new Vector(position);
    this.lastPosition = position.substract(initialVelocity.multiply(dt));
    this.model = model;
    this.modelInstance = modelInstance;
    this.acceleration = new Vector(0, 0, 0);
    this.currentAcceleration = new Vector(acceleration);
    this.minVelocity = minVelocity;
    this.maxVelocity = maxVelocity;
    this.maxJerk = maxJerk;
  }

  public static ArrayList<Vector> getDirections() {
    return Boid.directions;
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

    Vector steer = acceleration.substract(currentAcceleration);
    float steerMagnitude = steer.length();
    if (steerMagnitude > this.maxJerk) {
      steer = steer.normalize().multiply(maxJerk);
    }

    currentAcceleration = currentAcceleration.add(steer);

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
    Vector newPosition = position.add(displacement).add(currentAcceleration.multiply(dt * dt));

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

  public Vector avoidObstacles(ArrayList<Obstacle> obstacles, float lookAheadDistance) {
    Vector velocity = getVelocity(0.016f);
    if (velocity.length() == 0) return new Vector(0,0,0);

    Vector forward = velocity.normalize();
    float threshold = 0.58f;

    Ray forwardRay = new Ray(this.position, forward);

    boolean collisionImminent = false;
    for (Obstacle obs : obstacles) {
        float dist = forwardRay.getCollisionDistance(obs);
        if (dist != -1 && dist < lookAheadDistance) {
            collisionImminent = true;
            break;
        }
    }

    if (!collisionImminent) return new Vector(0,0,0);

    for (Vector dir : directions) {
        if (forward.dotProduct(dir) < threshold) continue;

        Ray ray = new Ray(this.position, dir);
        boolean blocked = false;

        for (Obstacle obs : obstacles) {
            float dist = ray.getCollisionDistance(obs);
            if (dist != -1 && dist < lookAheadDistance) {
                blocked = true;
                break;
            }
        }

        if (!blocked) {
            return dir.multiply(maxVelocity).substract(velocity);
        }
    }

    return new Vector(0,0,0);
  }

public Vector avoidObstacles(ArrayList<Obstacle> obstacles, float lookAheadDistance, float dt) {
    Vector velocity = getVelocity(dt);
    if (velocity.length() == 0) return new Vector(0,0,0);

    Vector forward = velocity.normalize();
    float threshold = 0.0f;

    Ray forwardRay = new Ray(this.position, forward);
    boolean collisionImminent = false;
    for (Obstacle obs : obstacles) {
        float dist = forwardRay.getCollisionDistance(obs);
        if (dist != -1 && dist < lookAheadDistance) {
            collisionImminent = true;
            break;
        }
    }

    if (!collisionImminent) return new Vector(0,0,0);

    for (Vector dir : directions) {
        if (forward.dotProduct(dir) < threshold) continue;

        Ray ray = new Ray(this.position, dir);
        boolean blocked = false;

        for (Obstacle obs : obstacles) {
            float dist = ray.getCollisionDistance(obs);
            if (dist != -1 && dist < lookAheadDistance) {
                blocked = true;
                break;
            }
        }

        if (!blocked) {
          return dir.multiply(maxVelocity).substract(velocity);
        }
    }

    return new Vector(0,0,0);
  }
}
