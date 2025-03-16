import { Link } from "react-router-dom";

const Routing = () => (
  <div className="page-container">
    <section className="hero-section">
      <h1 className="hero-title">Fastest Route Finder</h1>
      <p className="hero-subtitle">
        Explore advanced algorithms and technologies used in modern navigation systems
        that efficiently find optimal paths through complex networks.
      </p>
    </section>

    <div className="feature-grid">
      <div className="feature-card">
        <i className="fas fa-map-marked-alt fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
        <h2>Pathfinding Algorithms</h2>
        <p>Discover how algorithms like Dijkstra's, A*, and Bellman-Ford solve the shortest path problem.</p>
      </div>

      <div className="feature-card">
        <i className="fas fa-traffic-light fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
        <h2>Real-time Traffic</h2>
        <p>Learn how modern navigation systems incorporate live traffic data to optimize routes dynamically.</p>
      </div>

      <div className="feature-card">
        <i className="fas fa-truck fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
        <h2>Logistics Optimization</h2>
        <p>Explore how delivery companies use route optimization to save time, fuel, and reduce emissions.</p>
      </div>

      <div className="feature-card">
        <i className="fas fa-network-wired fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
        <h2>Network Routing</h2>
        <p>Understand how data packets find their way through the internet using routing protocols.</p>
      </div>
    </div>

    <section className="detail-section">
      <h2>Understanding Path Finding Algorithms</h2>
      <p>
        Path finding algorithms are computational procedures designed to find the shortest or optimal path 
        between two points in a network or graph. These algorithms are fundamental to modern navigation systems, 
        logistics planning, network routing, and even video game AI.
      </p>

      <div className="row align-items-center my-4">
        <div className="col-md-6">
          <img 
            src="data:image/jpeg;base64,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" 
            alt="Visualization of Dijkstra's algorithm finding the shortest path" 
            className="tech-image"
          />
        </div>
        <div className="col-md-6">
          <h3>Core Concepts</h3>
          <p>
            Path finding algorithms operate on graphs, which consist of:
          </p>
          <ul>
            <li><strong>Nodes</strong> (vertices): Representing locations or decision points</li>
            <li><strong>Edges</strong>: Connections between nodes, often with associated costs (distance, time, etc.)</li>
            <li><strong>Heuristics</strong>: Estimated costs used to guide the search in informed algorithms</li>
          </ul>
        </div>
      </div>

      <h2>Major Pathfinding Algorithms</h2>

      <div className="row">
        <div className="col-md-6">
          <h3>Dijkstra's Algorithm</h3>
          <p>
            Developed by Edsger W. Dijkstra in 1956, this algorithm guarantees finding the shortest path in a graph 
            with non-negative edge weights. It works by:
          </p>
          <ol>
            <li>Maintaining a set of unvisited nodes</li>
            <li>Assigning a tentative distance value to each node</li>
            <li>Repeatedly selecting the unvisited node with the smallest tentative distance</li>
            <li>Updating the tentative distances of its neighbors</li>
          </ol>
          <p>
            <span className="highlight">Time Complexity</span>: O(|V|²) with basic implementation, 
            or O(|E| + |V| log |V|) with priority queue optimization, where V is the number of vertices and E is the number of edges.
          </p>
        </div>
        <div className="col-md-6">
          <h3>A* (A-Star) Algorithm</h3>
          <p>
            A* extends Dijkstra's algorithm by incorporating heuristics to guide the search more efficiently 
            toward the goal. It evaluates nodes by combining:
          </p>
          <ul>
            <li><strong>g(n)</strong>: The cost to reach the node from the start</li>
            <li><strong>h(n)</strong>: The estimated cost from the node to the goal</li>
            <li><strong>f(n) = g(n) + h(n)</strong>: The total estimated cost of the path through node n</li>
          </ul>
          <p>
            A* is optimal if the heuristic is admissible (never overestimates the actual cost) and consistent 
            (satisfies the triangle inequality).
          </p>
        </div>
      </div>

      <div className="row mt-4">
        <div className="col-md-6">
          <h3>Bellman-Ford Algorithm</h3>
          <p>
            This algorithm finds the shortest paths from a source vertex to all other vertices, even in graphs 
            with negative edge weights. It can detect negative weight cycles and is often used in routing protocols.
          </p>
          <p>
            While slower than Dijkstra's algorithm (O(|V| × |E|) time complexity), it handles a broader range of scenarios.
          </p>
        </div>
        <div className="col-md-6">
          <h3>Floyd-Warshall Algorithm</h3>
          <p>
            This dynamic programming approach finds the shortest paths between all pairs of vertices in a weighted graph. 
            With O(|V|³) time complexity, it's efficient for dense graphs and when all-pairs shortest paths are needed.
          </p>
        </div>
      </div>

      <h2>Real-World Applications</h2>

      <div className="row">
        <div className="col-md-4">
          <div className="feature-card">
            <i className="fas fa-map-location-dot fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
            <h3>Navigation Systems</h3>
            <p>
              Google Maps, Waze, and other navigation apps use sophisticated route finding algorithms combined with 
              real-time traffic data to suggest optimal routes to users.
            </p>
          </div>
        </div>
        <div className="col-md-4">
          <div className="feature-card">
            <i className="fas fa-globe fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
            <h3>Internet Routing</h3>
            <p>
              Protocols like OSPF (Open Shortest Path First) and BGP (Border Gateway Protocol) use path finding 
              algorithms to route data efficiently across the internet.
            </p>
          </div>
        </div>
        <div className="col-md-4">
          <div className="feature-card">
            <i className="fas fa-gamepad fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
            <h3>Video Games</h3>
            <p>
              Game developers use A* and other algorithms to control NPC movement, allowing characters to navigate 
              game worlds intelligently and realistically.
            </p>
          </div>
        </div>
      </div>

      <h2>Advanced Topics in Route Optimization</h2>

      <div className="row mt-4">
        <div className="col-md-6">
          <h3>The Traveling Salesman Problem</h3>
          <p>
            This famous NP-hard problem seeks to find the shortest possible route that visits a set of cities exactly 
            once and returns to the origin. While no efficient exact algorithm exists for large instances, various 
            approximation algorithms and heuristics provide good solutions in practice.
          </p>
        </div>
        <div className="col-md-6">
          <h3>Vehicle Routing Problem</h3>
          <p>
            An extension of the Traveling Salesman Problem, this involves optimizing routes for multiple vehicles 
            with constraints like vehicle capacity, time windows, and pickup/delivery pairs. It's vital for logistics 
            companies seeking to minimize costs while meeting customer demands.
          </p>
        </div>
      </div>

      <div className="card p-4 mt-4" style={{ background: "rgba(59, 130, 246, 0.05)", borderRadius: "10px" }}>
        <h3>Python Implementation of Dijkstra's Algorithm</h3>
        <pre className="bg-dark text-light p-3 rounded">
          <code>
{`import heapq

def dijkstra(graph, start, end):
    # Initialize distances with infinity for all nodes except the start
    distances = {node: float('infinity') for node in graph}
    distances[start] = 0
    priority_queue = [(0, start)]
    previous = {node: None for node in graph}
    
    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)
        
        # If we've reached the end node
        if current_node == end:
            path = []
            while current_node:
                path.append(current_node)
                current_node = previous[current_node]
            return distances[end], path[::-1]
        
        # If we've found a longer path, skip
        if current_distance > distances[current_node]:
            continue
            
        # Check neighbors
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight
            
            # If we've found a shorter path
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous[neighbor] = current_node
                heapq.heappush(priority_queue, (distance, neighbor))
    
    return float('infinity'), []`}
          </code>
        </pre>
      </div>

      <div className="text-center mt-5">
        <Link to="/" className="btn btn-outline-primary btn-lg">
          <i className="fas fa-arrow-left me-2"></i>
          Back to Home
        </Link>
      </div>
    </section>
  </div>
);

export default Routing; 