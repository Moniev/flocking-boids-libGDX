package com.moniev.boids.core.Ray;

import java.util.ArrayList;

import com.moniev.boids.core.Boid.Boid;
import com.moniev.boids.core.Obstacle.Obstacle;
import com.moniev.boids.core.Vector.Vector;

public class Ray {
  public Vector origin;
  public Vector direction;

  public Ray(Vector origin, Vector direction) {
    this.origin = origin;
    this.direction = direction.normalize();
  }

  public Ray(Boid boid, Vector direction) {
    this.origin = boid.position;
    this.direction = direction.normalize();
  }

  public static ArrayList<Ray> fromBoid(Boid boid) {
    ArrayList<Ray> rays = new ArrayList<>();
    for (Vector direction : Boid.getDirections()) {
      Ray ray = new Ray(boid, direction.normalize());
      rays.add(ray);
    }

    return rays;
  }

  public boolean intersects(Obstacle obstacle) {
    return false;
  }

  public float getCollisionDistance(Obstacle obstacle) {
    Vector oc = origin.substract(obstacle.position);
    float b = 2.0f * oc.dotProduct(direction);
    float c = oc.dotProduct(oc) - obstacle.radius * obstacle.radius;
    float discriminant = b * b - 4 * c;

    if (discriminant < 0) return -1;

    float t = (-b - (float)Math.sqrt(discriminant)) / 2.0f;
    return (t > 0) ? t : -1;
  }
}

