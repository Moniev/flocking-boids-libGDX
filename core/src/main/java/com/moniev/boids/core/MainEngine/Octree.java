package com.moniev.boids.core.MainEngine;

import java.util.ArrayList;

import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector3;
import com.moniev.boids.core.Boid.Boid;
import com.moniev.boids.core.Vector.Vector;

public class Octree {
    public OctreeNode root;

    public final int maxDepth = 3;
    public int totalDepth = 1;

    public final float stepDt;
    public final Vector center;
    public final float minX, minY, minZ;
    public final float maxX, maxY, maxZ;
    private final Engine engine;
    private final float maxForce = 2.99f;
    private final float minForce = 2.99f; 
    private final float distanceBetween = 16f;
    private final float alignmentForce = 1.2f;
    private final float cohesionForce = 1.2f;
    private final float separationForce = 1.4f;
    public final float alignmentDistance = 16f;
    private final float cohesionDistance = 16f;
    private final float separationDistance = 16f;


    public Octree(Vector center, int size, int threads, ModelBuilder modelBuilder, float stepDt, Engine engine) {
        this.center = center;
        this.minX = center.x - size;
        this.minY = center.y - size;
        this.minZ = center.z - size;

        this.maxX = center.x + size;
        this.maxY = center.y + size;
        this.maxZ = center.z + size;

        this.stepDt = stepDt;

        this.root = new OctreeNode(center, size, size, maxDepth, null, modelBuilder, this);    
        this.engine = engine;
    }

    private Vector limitForce(Vector vector) {
        if(vector.length > maxForce) {
            return vector.normalize().multiply(maxForce);
        } else if(vector.length < minForce) {
            return vector.normalize().multiply(minForce);
        } else {
            return vector;
        }
    }

    private ArrayList<OctreeNode> getNearNodes(Vector position){
        ArrayList<OctreeNode> nearNodes = new ArrayList<>();
        findNearNodes(nearNodes, root, position);
        return nearNodes;
    }

    private void findNearNodes(ArrayList<OctreeNode> nearNodes, OctreeNode root, Vector position) {
        if(root == null) return;
        
        if(root.isLeaf && !root.boids.isEmpty()){
            if(root.center.distance(position) <= 10) {
                nearNodes.add(root);
            }
        } else {
            for(OctreeNode node : root.children) {
                findNearNodes(nearNodes, node, position);
            }
        }
    }

    public int countBoids(OctreeNode root) {
        if (root == null) return 0;
    
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
        if (node == null) return;

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

    public void updateBoids(OctreeNode root, float subStepDt) {
        if (root == null) return; 
        
        if(root.isLeaf) {
            for(Boid boid : root.boids) {
                boid.update(subStepDt);
            }
        } else {
            for(OctreeNode node : root.children) {
                updateBoids(node, subStepDt);
            }
        }
    }

    public ArrayList<Boid> getBorderBoids(ArrayList<OctreeNode> adjacentNodes) {
        ArrayList<Boid> borderBoids = new ArrayList<>();
        for(OctreeNode node : adjacentNodes) {
            for(Boid boid : node.boids) {
                if(node.isNearBorder(boid)) {
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
        if(root == null) return;

        if(root.isLeaf && root.isBorder) {
            borderNodes.add(root);
        }

        for(OctreeNode node : root.children) {
            findBorderNodes(node, borderNodes);
        }
    }

    public void renderBoids(ModelBatch modelBatch, OctreeNode root) {    
        if (root == null) return; 
    
        if (root.isLeaf) {
            for (Boid boid : root.boids) {
                Vector _target = boid.position.add(boid.getVelocity(engine.subStepDt)); 
                Vector3 target = new Vector3(_target.x, _target.y, _target.z).nor();
                
                Quaternion rotation = new Quaternion();
                rotation.setFromCross(Vector3.Y, target);

                boid.modelInstance.transform.set(rotation);
                boid.modelInstance.transform.setTranslation(
                    boid.position.x, 
                    boid.position.y, 
                    boid.position.z
                );
                modelBatch.render(boid.modelInstance);
            }
        } else {
            for (OctreeNode node : root.children) {           
                renderBoids(modelBatch, node);
            }
        }
    }
    
    public void disposeBoids(OctreeNode root) {
        if (root == null) return; 

        if(root.isLeaf) {
            for(Boid boid : root.boids) {
                boid.model.dispose();
            }
        } else {
            for(OctreeNode node : root.children) {
                disposeBoids(node);
            }
        }
    }

    public void disposeTree(OctreeNode root) {
        if (root == null) return; 

        if(root.isLeaf) {
            root.model.dispose();
        } else {
            for(OctreeNode node : root.children) {
                disposeTree(node);
            }
        }
    }

    public void renderTree(ModelBatch modelBatch, OctreeNode root) {
        if (root == null) return; 

        if(root.isLeaf) {
            modelBatch.render(root.modelInstance);
        } else {
            for(OctreeNode node : root.children){
                renderTree(modelBatch, node);
            }
        }
    }

    public Vector calculateCohesion(Boid boid, OctreeNode node) {
        Vector cohesion = new Vector(0, 0, 0);
        for(Boid other : node.boids) {
            if(other != boid) {
                float distance = boid.position.distance(other.position); 
                if(distance > 0 && distance <= cohesionDistance) {
                    cohesion.set(cohesion.add(other.position));
                }
            }
        }

        if(!node.boids.isEmpty()) {
            return cohesion.subdivide(node.boids.size());
        }

        return limitForce(cohesion);
    }

    public Vector calculateAlignment(Boid boid, OctreeNode node) {
        Vector alignment = new Vector(0, 0, 0);
        for(Boid other : root.boids) {
            if(other != boid) {
                float distance = boid.position.distance(other.position);
                if(distance > 0 && distance <= distanceBetween ) {
                    alignment.set(alignment.add(boid.getVelocity(engine.subStepDt)));
                }
            }
        }

        if(!node.boids.isEmpty()) {
            return alignment.subdivide(node.boids.size());
        }

        return limitForce(alignment);
    }

    public Vector calculateSeparation(Boid boid, OctreeNode node) {
        Vector separation = new Vector(0, 0, 0);
        for(Boid other : root.boids) {
            float distance = boid.position.distance(other.position);
            if(distance > 0 && distance <= separationDistance) {
                separation.set(separation.add(other.position.substract(boid.position)));
            }
        }

        if(!node.boids.isEmpty()) {
            return separation.subdivide(node.boids.size());
        }

        return limitForce(separation);
    }

    public void resolveOuterAdjustment(OctreeNode root) {
        if(root == null) return;

        if(root.isLeaf) {
            ArrayList<OctreeNode> adjacentNodes = root.getAdjacentNodes(this.root);
            
            if(!adjacentNodes.isEmpty()) {
                for(Boid inner : root.boids) {
                    inner.acceleration.set(0);
                    
                    Vector alignment = new Vector(0, 0, 0);
                    Vector cohesion = new Vector(0, 0, 0);
                    Vector separation = new Vector(0, 0, 0);
                        for(OctreeNode node : adjacentNodes) {
                            alignment.set(alignment.add(calculateAlignment(inner, node)));
                            cohesion.set(cohesion.add(calculateCohesion(inner, node)));
                            separation.set(separation.add(calculateSeparation(inner, root)));
                        }
                        
                        inner.accelerate(limitForce(alignment.substract(inner.getVelocity(engine.subStepDt)).multiply(alignmentForce)));
                        inner.accelerate(limitForce(cohesion.substract(inner.position).multiply(cohesionForce)));
                        inner.accelerate(limitForce(separation.multiply(separationForce)));
                }
            }
        }

        for(OctreeNode node : root.children) {
            resolveOuterAdjustment(node);
        }
    }

    public void resolveInnerAdjustment(OctreeNode root) {
        if(root == null) return;

        if(root.isLeaf) {
            for(Boid boid : root.boids) {
                boid.acceleration.set(0);
                Vector alignment = calculateAlignment(boid, root);
                Vector cohesion = calculateCohesion(boid, root);
                Vector separation = calculateSeparation(boid, root);

                boid.accelerate(alignment.substract(boid.getVelocity(engine.subStepDt)).multiply(alignmentForce));
                boid.accelerate(cohesion.substract(boid.position).multiply(cohesionForce));
                boid.accelerate(separation.multiply(separationForce));
            }
        } else {
            for(OctreeNode node : root.children) {
                resolveInnerAdjustment(node);
            }
        }
    }

    public void updateSpatialLookup(OctreeNode node) {
        if (node == null) return;
    
        if (node.isLeaf) {
            ArrayList<Boid> boidsToMove = new ArrayList<>();
            for (Boid boid : node.boids) {
                OctreeNode target = findTargetNode(root, boid);
                if (target != node) { 
                    boidsToMove.add(boid);
                }
            }
            
            for (Boid boid : boidsToMove) {
                node.boids.remove(boid); 
                OctreeNode target = findTargetNode(root, boid);
                if (target != null) {
                    target.insert(boid); 
                }
            }

        } else {
            for (OctreeNode child : node.children) {
                updateSpatialLookup(child);
            }
        }
    }

    public OctreeNode findTargetNode(OctreeNode root, Boid boid) {    
        if (root.isLeaf) return root;
        int i = root.getChildIndex(boid.position);
        return findTargetNode(root.children[i], boid);
    }
}
