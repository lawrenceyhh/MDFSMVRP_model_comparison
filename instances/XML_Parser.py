# define function to download data and unzip data
import numpy as np

class XML_Parser:
    def __init__(self, name):
        self.name = name
        self.url = 'http://comopt.ifi.uni-heidelberg.de/software/TSPLIB95/XML-TSPLIB/instances/' + name + '.xml.zip'
        self.zip = 'instances\\' + str(self.name) + '.xml.zip'
        self.xml = 'instances\\' + str(self.name) + '.xml'
        
   # define function to download data and unzip data
    def load_data(self):
        """
        Function to download and unzip data
        Arguments:
            url: link to download file
            filename: defined name to save data downloaded
        Return: None
        The file download will be saved into the working directory and unzip here.
        """
        
        import urllib.request
        from zipfile import ZipFile
        
        _ = urllib.request.urlretrieve(self.url, self.zip)
        
        #unzip the file downloaded
        with ZipFile(self.zip, 'r') as zip_ref:
            zip_ref.extractall("instances")
            
    # define fuction to read data from xml file
    def import_data(self):
        """
        Function to read the xml file using ElementTree package from Python standard library
        Arguments:
            filename: name of the xml file to parse
        Return:
            xroot: An Element tree from the xml file
        """
        
        import xml.etree.ElementTree as et 
        xtree = et.parse(self.xml)
        xroot = xtree.getroot()
        
        return xroot

    # define function to create distance matrix
    def dist_matrix(self, xroot):
        """
        Function to create the distance matrix from the Element tree parsed from xml file
        Arguments:
            xroot: The Element tree parsed from xml file
        Return:
            distance: 2-d Numpy array of distance between locations indexing as [0, 1, ...]
        """     
        
        #create distance matrix
        cities = len(xroot.findall("./graph/vertex"))
        distance = np.zeros((cities, cities))
        
        from_node = 0
        for child in xroot.iter('vertex'):
            for child1 in child:
                dist = float(child1.attrib.get('cost'))
                to_node = int(child1.text)
                distance[from_node, to_node] = dist

            from_node += 1
        
        max_distance = np.nanmax(distance)
        for i in range(cities):
            distance[i, i] = max_distance * 10000 #very large number for distance to itself => no revisited 

        return distance


    def gen_dataset(self, seed=None):
        """
        Function to load the TSP dataset from the TSPLIB of University of Heidelberg 
        (http://comopt.ifi.uni-heidelberg.de/software/TSPLIB95/XML-TSPLIB/instances/) 
        and covert it to a multi-depot fleet size and mix vehicle routing dataset.

        Returns:
            demands: a dictionary storing the demand of each node
            vertices: a list of all nodes including depots and customers
            arcs: a dictionary storing the distance of all the arcs between two nodes
            V_d: a list of all depot nodes
            V_c: a list of all customer nodes
            F: fixed cost for different types of vehicles 
            alpha: variable costs associate with the distance traveled by different types of vehicles
            K: different types of vehicles
            Q: capacities of different types of vehicles
        """
        
        import random
        
        if seed is not None:
            random.seed(seed)
        
        self.load_data()
        xroot = self.import_data()
        distance = self.dist_matrix(xroot)
        vertices = list(range(len(distance)))
        
        V_d = [0]
        # convert two customer nodes to depots (to make the dataset mutidepot)
        depot_one, depot_two = random.choices(vertices, k=2)
        V_d.append(depot_one)
        V_d.append(depot_two)
        for i in V_d:
            for j in V_d:
                distance[i, j] = np.nanmax(distance) * 10000
        
        V_c = [x for x in vertices if x not in V_d]
        
        # generate a demand between 1 to 3 for each customer in V_c
        demands = {i: random.randint(1, 3) if i in V_c else 0 for i in vertices}
        
        arcs = {(i, j): distance[i, j] for i in vertices for j in vertices}
        
        # assume there are three types of vehicles
        K = list(range(3))
        
        # set the parameters for all three types of vehicles
        # fixed cost
        F = [200, 250, 300]
        # variable cost
        alpha = [3, 2.75, 2.5]
        # capacity
        Q = [6, 8, 10]                
    
        return demands, vertices, arcs, V_d, V_c, F, alpha, K, Q