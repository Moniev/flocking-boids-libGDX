package com.moniev.boids.core.Ray;

import java.util.ArrayList;

import com.moniev.boids.core.Boid.Boid;
import com.moniev.boids.core.Obstacle.Obstacle;
import com.moniev.boids.core.Vector.Vector;

class Ray {
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
}
