package org.sunflow.core.accel;

import java.util.Arrays;
import java.util.Comparator;

import org.sunflow.core.AccelerationStructure;
import org.sunflow.core.IntersectionState;
import org.sunflow.core.PrimitiveList;
import org.sunflow.core.Ray;
import org.sunflow.math.BoundingBox;
import org.sunflow.system.UI;
import org.sunflow.system.UI.Module;

public class BVH implements AccelerationStructure {
	BVH_Node root = new BVH_Node();
	PrimitiveList p;
	
	//Class for coupling ID with bounding box
	public class Prim{
		int ID;
		BoundingBox idBox;
		
		Prim(int ID) //constructorrrr
		{
			this.ID = ID;
			idBox = new BoundingBox();
		}
	}
	public class BVH_Node{
	BVH_Node leftChild; 
	BVH_Node rightChild;
	BoundingBox bbox;
	Prim[] shapes;
	
	
		BVH_Node() 
		{
			bbox = new BoundingBox();
			shapes=null;
			leftChild=null;
			rightChild=null;
		}
	}
	
	BVH_Node recursivebuild (Prim[] primBoxes, int start, int end, int axis) {
		if (end-start <= 0)
		{
			return null;
		}
		
		BVH_Node node = new BVH_Node();
		if ((end - start)<=4) //if we are at the bottom of tree, only 4 or less primitive make leaf node
		{ 
			node.shapes = new Prim[end-start];
			for (int leaves=0; leaves<end-start; leaves++)
			{
				node.shapes[leaves] = primBoxes[start+leaves];
				node.bbox.include(node.shapes[leaves].idBox);
			}
			return node;
		}
			
			//get bounding box of each primitive (use simple for loop)
			//add it to the node
		
		else{
			//find the median split:
	        switch (axis) {
            case 0://Sort x and creates a comparator for Array.sort on x
            	Arrays.sort(primBoxes, start, end, new Comparator<Prim>(){
					@Override
					public int compare(Prim arg0, Prim arg1) {
						// TODO Auto-generated method stub
						return Float.compare(arg0.idBox.getCenter().x , arg1.idBox.getCenter().x);
					}});
                break;
            case 1://y
            	Arrays.sort(primBoxes, start, end, new Comparator<Prim>(){
					@Override
					public int compare(Prim arg0, Prim arg1) {
						// TODO Auto-generated method stub
						return Float.compare(arg0.idBox.getCenter().y , arg1.idBox.getCenter().y);
					}});
                break;
            case 2://z
            	Arrays.sort(primBoxes, start, end, new Comparator<Prim>(){
					@Override
					public int compare(Prim arg0, Prim arg1) {
						// TODO Auto-generated method stub
						return Float.compare(arg0.idBox.getCenter().z , arg1.idBox.getCenter().z);
					}});
                break;
	        }
//				sort A[start..end) along axis, and split the
				int m =(start+end)/2;
				node.leftChild = recursivebuild(primBoxes, start, m+1, (axis+1)%3);
				node.rightChild = recursivebuild(primBoxes, m+1, end, (axis+1)%3);
				node.bbox.include(node.leftChild.bbox);
				node.bbox.include(node.rightChild.bbox);
		}
	
		return node;
		}
	
	
	void BVH_Intersect(Ray r, BVH_Node node, IntersectionState state) {
		if (node == null)
		{
			return;
		}
		if (node.shapes != null)
		{
			for (int index=0; index < node.shapes.length; index++)
				p.intersectPrimitive(r, node.shapes[index].ID, state);
				
		//intersect(r, node.shapes[index].ID);
		}
		else
		{
			  float intervalMin = r.getMin();
		        float intervalMax = r.getMax();
		        float orgX = r.ox;
		        float dirX = r.dx, invDirX = 1 / dirX;
		        float t1, t2;
		        t1 = (node.bbox.getMinimum().x - orgX) * invDirX;
		        t2 = (node.bbox.getMaximum().x - orgX) * invDirX;
		        if (invDirX > 0) {
		            if (t1 > intervalMin)
		                intervalMin = t1;
		            if (t2 < intervalMax)
		                intervalMax = t2;
		        } else {
		            if (t2 > intervalMin)
		                intervalMin = t2;
		            if (t1 < intervalMax)
		                intervalMax = t1;
		        }
		        if (intervalMin > intervalMax)
		            return;
		        float orgY = r.oy;
		        float dirY = r.dy, invDirY = 1 / dirY;
		        t1 = (node.bbox.getMinimum().y - orgY) * invDirY;
		        t2 = (node.bbox.getMaximum().y - orgY) * invDirY;
		        if (invDirY > 0) {
		            if (t1 > intervalMin)
		                intervalMin = t1;
		            if (t2 < intervalMax)
		                intervalMax = t2;
		        } else {
		            if (t2 > intervalMin)
		                intervalMin = t2;
		            if (t1 < intervalMax)
		                intervalMax = t1;
		        }
		        if (intervalMin > intervalMax)
		            return;
		        float orgZ = r.oz;
		        float dirZ = r.dz, invDirZ = 1 / dirZ;
		        t1 = (node.bbox.getMinimum().z - orgZ) * invDirZ;
		        t2 = (node.bbox.getMaximum().z - orgZ) * invDirZ;
		        if (invDirZ > 0) {
		            if (t1 > intervalMin)
		                intervalMin = t1;
		            if (t2 < intervalMax)
		                intervalMax = t2;
		        } else {
		            if (t2 > intervalMin)
		                intervalMin = t2;
		            if (t1 < intervalMax)
		                intervalMax = t1;
		        }
		        if (intervalMin > intervalMax)
		            return;
		        
		BVH_Intersect(r, node.leftChild, state);
		BVH_Intersect(r, node.rightChild, state);
		}
		}
	
	@Override
	public void build(PrimitiveList primitives) {
		p = primitives;
		//Create Object Array for Primitive Index
		Prim[] primBoxes = new Prim [primitives.getNumPrimitives()];
		for (int index=0; index<primitives.getNumPrimitives(); index++)
		{
			 float minx = primitives.getPrimitiveBound(index, 0);
			 float maxx = primitives.getPrimitiveBound(index, 1);
			 float miny = primitives.getPrimitiveBound(index, 2);
			 float maxy = primitives.getPrimitiveBound(index, 3);
			 float minz = primitives.getPrimitiveBound(index, 4);
			 float maxz = primitives.getPrimitiveBound(index, 5);
			 primBoxes[index] = new Prim(index);
			 
			 primBoxes[index].idBox.include(maxx, maxy, maxz);
			 primBoxes[index].idBox.include(minx, miny, minz);
		}
		
	//Call recursive Build and send it the Array of Primitive Objects, First Prim, Last Prim, and Axis
	root = recursivebuild(primBoxes, 0, primBoxes.length, 0);
	
	System.out.print(primitives.getNumPrimitives());
		// TODO Auto-generated method stub
		//call my recursive build function

	}

	@Override
	public void intersect(Ray r, IntersectionState istate) {
		//send info to BVH_intersect, root node, bounding box of the node
      BVH_Intersect(r, root, istate);
		// TODO Auto-generated method stub

	}

}
