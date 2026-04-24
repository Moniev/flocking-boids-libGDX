package com.moniev.boids.core.MainEngine;

import java.util.ArrayList;

import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.moniev.boids.core.Boid.Boid;
import com.moniev.boids.core.Vector.Vector;

public class Octree {
  public OctreeNode root;

  public final int maxDepth = 8;
  public int totalDepth = 1;

  public final float stepDt;
  public final Vector center;

  public final float minX, minY, minZ;
  public final float maxX, maxY, maxZ;

  public final Engine engine;

  private final float maxForce = 12f;
  private final float minForce = 1f;

  private final float distanceBetween = 100f;

  private final float alignmentForce = 0.5f;
  private final float cohesionForce = 2f;
  private final float separationForce = 0.7f;

  private final float turnForce = 50f;
  private final float turnDistance = 120f;

  public Octree(Vector center, int size, int boidsLimit, int threads, ModelBuilder modelBuilder, float stepDt,
      Engine engine) {
    this.engine = engine;
    this.center = center;
    this.minX = center.x - size;
    this.minY = center.y - size;
    this.minZ = center.z - size;

    this.maxX = center.x + size;
    this.maxY = center.y + size;
    this.maxZ = center.z + size;

    this.stepDt = stepDt;

    this.root = new OctreeNode(center, size, boidsLimit, maxDepth, null, this);
  }

  private Vector limitForce(Vector vector) {
    float length = vector.length();

    if (length == 0) {
      return new Vector(0, 0, 0);
    }

    if (length > maxForce) {
      return vector.normalize().multiply(maxForce);
    } else if (length < minForce) {
      return vector.normalize().multiply(minForce);
    }

    return vector;
  }

  public int countBoids(OctreeNode root) {
    if (root == null)
      return 0;

    int count = 0;
    if (root.isLeaf) {
      count += root.boids.size();
    } else {
      for (OctreeNode child : root.children) {
        count += countBoids(child);
      }
    }

    return count;
  }

  public ArrayList<OctreeNode> getLeafs() {
    ArrayList<OctreeNode> leafs = new ArrayList<>();
    collectLeafs(root, leafs);
    return leafs;
  }

  public void collectLeafs(OctreeNode node, ArrayList<OctreeNode> leafNodes) {
    if (node == null)
      return;

    if (node.isLeaf) {
      leafNodes.add(node);
    } else {
      for (OctreeNode child : node.children) {
        collectLeafs(child, leafNodes);
      }
    }
  }

  public void addBoid(Boid boid) {
    root.insert(boid);
  }

  public void calculateForces(Boid boid) {
    OctreeNode currentNode = findTargetNode(this.root, boid);
    ArrayList<OctreeNode> nodesToSearch = currentNode.getAdjacentNodes();
    nodesToSearch.add(currentNode);

    ArrayList<Boid> neighbors = new ArrayList<>();
    for (OctreeNode node : nodesToSearch) {
      for (Boid other : node.boids) {
        if (other != boid) {
          float distance = boid.position.distance(other.position);
          if (distance > 0 && distance <= distanceBetween) {
            neighbors.add(other);
          }
        }
      }
    }

    Vector avoidance = calculateBoundaryAvoidance(boid);
    boid.accelerate(avoidance);

    Vector obstacleAvoidance = boid.avoidObstacles(engine.getObstacles(), turnDistance, stepDt);
    if (obstacleAvoidance.length() > 0) {
        boid.accelerate(limitForce(obstacleAvoidance.multiply(turnForce * 2)));
    }

    if (neighbors.isEmpty()) {
      return;
    }

    Vector totalAlignment = new Vector(0, 0, 0);
    Vector totalCohesion = new Vector(0, 0, 0);
    Vector totalSeparation = new Vector(0, 0, 0);

    for (Boid other : neighbors) {
      totalAlignment = totalAlignment.add(other.getVelocity(stepDt));
      totalCohesion = totalCohesion.add(other.position);
      Vector diff = boid.position.substract(other.position);
      totalSeparation = totalSeparation.add(diff.subdivide(diff.dotProduct(diff)));
    }

    Vector cohesionVec = totalCohesion.subdivide(neighbors.size()).substract(boid.position);
    Vector separationVec = totalSeparation;
    Vector alignmentVec = totalAlignment.subdivide(neighbors.size()).substract(boid.getVelocity(stepDt));

    boid.accelerate(limitForce(alignmentVec.multiply(alignmentForce)));
    boid.accelerate(limitForce(cohesionVec.multiply(cohesionForce)));
    boid.accelerate(limitForce(separationVec.multiply(separationForce)));
  }

  private Vector calculateBoundaryAvoidance(Boid boid) {
    Vector force = new Vector(0, 0, 0);

    if (boid.position.x < minX + turnDistance) {
      force.x = turnForce;
    } else if (boid.position.x > maxX - turnDistance) {
      force.x = -turnForce;
    }

    if (boid.position.y < minY + turnDistance) {
      force.y = turnForce;
    } else if (boid.position.y > maxY - turnDistance) {
      force.y = -turnForce;
    }

    if (boid.position.z < minZ + turnDistance) {
      force.z = turnForce;
    } else if (boid.position.z > maxZ - turnDistance) {
      force.z = -turnForce;
    }

    return force;
  }

  public ArrayList<Boid> getBorderBoids(ArrayList<OctreeNode> adjacentNodes) {
    ArrayList<Boid> borderBoids = new ArrayList<>();
    for (OctreeNode node : adjacentNodes) {
      for (Boid boid : node.boids) {
        if (node.isNearBorder(boid)) {
          borderBoids.add(boid);
        }
      }
    }
    return borderBoids;
  }

  public ArrayList<OctreeNode> getBorderNodes() {
    ArrayList<OctreeNode> borderNodes = new ArrayList<>();
    findBorderNodes(root, borderNodes);
    return borderNodes;
  }

  private void findBorderNodes(OctreeNode root, ArrayList<OctreeNode> borderNodes) {
    if (root == null)
      return;

    if (root.isLeaf && root.isBorder) {
      borderNodes.add(root);
    }

    for (OctreeNode node : root.children) {
      findBorderNodes(node, borderNodes);
    }
  }

  public void renderTree(ModelBatch modelBatch, OctreeNode root) {
    if (root == null)
      return;

    if (root.isLeaf) {
      modelBatch.render(root.modelInstance);
    } else {
      for (OctreeNode node : root.children) {
        renderTree(modelBatch, node);
      }
    }
  }

  public OctreeNode findTargetNode(OctreeNode root, Boid boid) {
    if (root.isLeaf) {
      return root;
    }
    int i = root.getChildIndex(boid.position);

    if (root.children[i] == null) {
      return root;
    }

    return findTargetNode(root.children[i], boid);
  }
}
