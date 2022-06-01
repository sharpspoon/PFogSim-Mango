package edu.auburn.pFogSim.clustering;

import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.HashMap;

import edu.auburn.pFogSim.netsim.*;
import edu.auburn.pFogSim.util.DataInterpreter;
import edu.boun.edgecloudsim.utils.Location;


/**
 * 
 * @author rcw0024
 *
 */
public class DBSCAN {
	private static final int MAX_FOG_LAYERS = 7;
	private ArrayList<FogCluster> clusterList = new ArrayList<FogCluster>();
	public int[][] parentCluster;
	
	
	/**
	 * Constructor
	 * @param nodes
	 */
	public DBSCAN(ArrayList<NodeSim> nodes) {
		
		HashMap<Integer, ArrayList<Location>> levelMap = new HashMap<Integer, ArrayList<Location>>();
		int level = 1000;
		double x_pos = -1.0, y_pos = -1.0, a_pos = -1.0;
		int[] clusterCount = {100, 40, 20, 10, 3, 1, 1}; // Shaik added - number of clusters to be created in a given layer.
		double[] maxLatency = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0}; // Shaik added - max latency between any two nodes in a given layer is 2 msec.
		double[] maxDistance = {200, 500, 750, 1250, 1000, 3000, 4000}; // Shaik added - max distance between any two nodes in a given layer is 2 msec.
		
		ArrayList<Location> newList = new ArrayList<>();
		for(int r = 1; r <= MAX_FOG_LAYERS; r++)
			levelMap.put(r, new ArrayList<Location>()); 
		//Add all nodes to levelMap based on their level values
		for(NodeSim node : nodes)
		{
			level = node.getLevel();
			x_pos = node.getLocation().getXPos();
			y_pos = node.getLocation().getYPos();
			a_pos = node.getLocation().getAltitude();
			
			Location pair = new Location(x_pos, y_pos, a_pos);
			if (levelMap.containsKey(level)) {
				levelMap.get(level).add(pair);
			}
			else {
				newList = new ArrayList<>();
				newList.add(pair);
				levelMap.put(level, newList);
			}
		}

		//Create clusters of nodes at each fog layer & save the configuration in clusterList
		for(int levelIter = 1; levelIter <= MAX_FOG_LAYERS; levelIter++)
		{
			//FogCluster fc = new FogCluster(levelMap.get(levelIter), levelIter, clusterCount[levelIter-1]); // Create specified number of clusters in each layer.
			//FogCluster fc = new FogCluster(levelMap.get(levelIter), levelIter, maxLatency[levelIter-1]); // Clusters defined by maximum latency among members of each cluster.
			FogCluster fc = new FogCluster(levelMap.get(levelIter), levelIter, maxDistance[levelIter-1]); // Clusters defined by maximum latency among members of each cluster.

			clusterList.add(fc);
		}
		
		//Identify parent/child relationships among clusters belonging to adjacent layers.
		makeClusters();
		
		//Log the configuration 
		writeClustersToFile();
	}// end FogHierCluster()
	
	
	/**
	 * Create HAFA Architecture inter-layer links
	 * @author Shaik
	 */
	private void makeClusters() 
	{
		double distance = 0;
		double clusterMaxDistance = 0 ;
		double minDistance = Double.MAX_VALUE;
		parentCluster = new int[clusterList.size()][];
		int j;

		//Now, for each set of clusters in adjacent layers, repeat the following:
				//Say clusters in layer-3 & layer-4
		
		for(int i = clusterList.size() - 2; i > 0; i--)
		{
			//entry pairs considered are (i-j) = 5-4, 4-3, 3-2, 2-1, 1-0
			//corresponding layer pairs considered are = 6-5, 5-4, 4-3, 3-2, 2-1
			
				j = i - 1; //i - upper layer index; j-lower layer index
				
				// clusterList element at index p has clusters info for layer p+1
				int clusterNumber3 = clusterList.get(j).getCluster().length;
				int clusterNumber4 = clusterList.get(i).getCluster().length;
				
				parentCluster[j] = new int[clusterNumber3];
				
				Double[][][] clusterSet3 = clusterList.get(j).getCluster();
				Double[][][] clusterSet4 = clusterList.get(i).getCluster();
				//parentCluster = new int[clusterNumber3];
				
				//For each cluster in lower layer, do the following
				for (int cLower=0; cLower<clusterNumber3; cLower++){
					minDistance = Double.MAX_VALUE;
					
					//For each cluster in upper layer, do the following
					for(int cUpper=0; cUpper<clusterNumber4; cUpper++){
						
						clusterMaxDistance = 0;
						//Calculate the ('max' for CompleteLink) distance between cluster from lower layer 'cLower'
						//and cluster from higher layer 'cUpper'
						// i.e. find the distance between each point of 'cLower' cluster 
						// and each point of 'cUpper' cluster
						// Note the maximum distance
						
						//From each point of 'cLower' cluster
						for (int cLoweri=0; cLoweri<clusterSet3[cLower].length; cLoweri++){
							// Get point coordinates
							double x1 = clusterSet3[cLower][cLoweri][0];
							double y1 = clusterSet3[cLower][cLoweri][1];
							double a1 = clusterSet3[cLower][cLoweri][2];
							
							//To each point of 'cUpper' cluster
							for (int cUpperj=0; cUpperj<clusterSet4[cUpper].length; cUpperj++){
								// Get point coordinates
								double x2 = clusterSet4[cUpper][cUpperj][0];
								double y2 = clusterSet4[cUpper][cUpperj][1];
								double a2 = clusterSet4[cUpper][cUpperj][2];
														
								//find the distance
								//distance = Math.sqrt(((x2-x1)*(x2-x1)) + ((y2-y1)*(y2-y1)));
								distance = DataInterpreter.measure(x1, y1, a1, x2,y2,a2);
								//System.out.println(distance);
								
								// Save the maximum distance
								if (distance > clusterMaxDistance){
									clusterMaxDistance = distance;
								}
								
							}// end for cUpperj
						}// end for cLoweri

						//If this is the closer upper layer cluster, then this is a better parent cluster
						if (clusterMaxDistance < minDistance){
							minDistance = clusterMaxDistance;
							parentCluster[j][cLower] = cUpper; 
						}
						
					}// end for cUpper
				}// end for cLower
				
				
				//Print Parent/Child relationships
				System.out.println("\nChildCluster"+"   "+"ParentCluster");
				for (int cLower=0; cLower<clusterNumber3; cLower++){
					System.out.println("         "+cLower+"   "+"         "+parentCluster[j][cLower]);
				}// end for cLower-Print
				
		}
		
		// Assign parent Puddle Ids for fog level 6 (city center) and fog level7 (cloud).
		parentCluster[5] = new int[1];
		parentCluster[5][0] = 0;
		parentCluster[6] = new int[1];
		parentCluster[6][0] = -1;

	}// end makeClusters()
	
	
	/**
	 * Return list of clusters.
	 * @return
	 */
	public ArrayList<FogCluster> getClusters() {
		return clusterList;
	}

	
	/**
	 * @return the clusterList
	 */
	public ArrayList<FogCluster> getClusterList() {
		return clusterList;
	}

	
	/**
	 * @param clusterList the clusterList to set
	 */
	public void setClusterList(ArrayList<FogCluster> clusterList) {
		this.clusterList = clusterList;
	}
	
	
	/**
	 * Save cluster info to file in XML format.
	 */
	public void writeClustersToFile() {
		PrintWriter writer;
		try {
			for (int i = 0; i < clusterList.size(); i++ ) {
				int level = i + 1;
				writer = new PrintWriter("sim_results/Clusters/level" + level + ".xml");
				writer.println("<?xml version=\"1.0\"?>");
				Double[][][] cluster = clusterList.get(i).getCluster();
				for (int j = 0; j < cluster.length; j++) {
					writer.println("<cluster"+ j +">");
					for (int k = 0; k < cluster[j].length; k++) {
						writer.println("<location>");
						writer.println("<x_pos>" + cluster[j][k][0] + "</x_pos>");
						writer.println("<y_pos>" + cluster[j][k][1] + "</y_pos>");
						writer.println("<altitude>" + cluster[j][k][2] + "</altitude>");
						writer.println("</location>");
					}
					writer.println("</cluster"+ j + ">");
				}
				writer.close();
			}
		}
		catch(Exception e) {
			
		}
	}
}