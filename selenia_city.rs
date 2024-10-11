use std::io;
use std::fmt;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::cmp::Ordering;
use std::hash::{Hash, Hasher};

macro_rules! parse_input {
    ($x:expr, $t:ident) => ($x.trim().parse::<$t>().unwrap())
}

const PRICE_TP: i32 = 5000;
const POD_CONSTRUCTION: i32 = 1000;
const MAX_TUBES_PER_BUILDING: i32 = 5;

enum Action {
    Tube { from: i32, to: i32 },
    Upgrade  { from: i32, to: i32 },
    Teleport  { entrance: i32, exit: i32 },
    Pod  { pod_id: i32, stops: Vec<i32> },
    Destroy  { pod_id: i32 },
    Wait,
}

impl fmt::Display for Action {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Action::Tube { from, to} => write!(f, "TUBE {} {};", from, to),

            Action::Upgrade { from, to } => write!(f, "UPGRADE {} {};", from, to),

            Action::Teleport { entrance, exit} => write!(f, "TELEPORT {} {};", entrance, exit),

            Action::Pod { pod_id, stops } => {
                let stops_str = stops.iter().map(|s| s.to_string()).collect::<Vec<_>>().join(" ");
                write!(f, "POD {} {};", pod_id, stops_str)
            }
            Action::Destroy { pod_id } => write!(f, "DESTROY {};", pod_id),

            Action::Wait => write!(f, "WAIT;")
        }
    }
}


#[derive(Debug, Copy, Clone, PartialEq)]
struct State {
    cost: f64,
    position: i32,
}

// Ord pour que le BinaryHeap soit un min-heap
impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.partial_cmp(&self.cost).unwrap() // inverser pour avoir un min-heap
    }
}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Eq for State {}


// En commun entre plusieurs structures
trait Getter {
    fn get_id(&self) -> i32;
    fn get_type(&self) -> i32;
    fn get_position(&self) -> (i32, i32);
}

// Capsule de transport
#[derive(Clone)]
struct Pods {
    id: i32,
    pod_stops: i32,
    capacity: i32, // nb de passagers 
    itinerary: Vec<i32>,
}

impl Pods {

    fn get_id(&self) -> i32 {
        self.id
    }

    fn get_pod_stops(&self) -> i32 {
        self.pod_stops
    }

    fn get_capacity(&self) -> i32 {
        self.capacity
    }

    fn get_itinerary(&self) -> Vec<i32> {
        self.itinerary.clone()
    }
 
    pub fn update_itinerary(&mut self, new_itinerary: Vec<i32>) {
        self.itinerary = new_itinerary;
    }
    
}

// Routes: Tube ou TP
#[derive(Clone, Debug, Hash, Eq, PartialEq)]
struct Routes {
    id_building_in: i32, // id entre 
    id_building_out: i32, // id sortie
}

impl Routes {
    // Normaliser la route en s'assurant que le plus petit id est toujours en premier
    fn new(id_building_in: i32, id_building_out: i32) -> Self {
        if id_building_in < id_building_out {
            Self { id_building_in, id_building_out }
        } else {
            Self { id_building_in: id_building_out, id_building_out: id_building_in }
        }
    }
}

// Aire d'atterissage
#[derive(Clone)] 
struct LandingArea {
    id: i32, 
    position: (i32, i32),
    nb_astronaut: i32,
    astronauts_by_types: HashMap<i32, i32>, 
}

impl Getter for LandingArea {
    fn get_id(&self) -> i32 {
        self.id
    }

    fn get_type(&self) -> i32 {
        0
    }

    fn get_position(&self) -> (i32, i32) {
        self.position
    }
}

// station lunaire
#[derive(Clone)] 
struct MoonArea {
    id: i32,
    type_moon_area: i32, // entre 1 et 20 et correspond a l'endroit ou les astronautes doivent atterir
    position: (i32, i32), 
}

impl Getter for MoonArea {
 
    fn get_id(&self) -> i32 {
        self.id
    }

    fn get_type(&self) -> i32 {
        self.type_moon_area
    }

    fn get_position(&self) -> (i32, i32) {
        self.position
    }
}

#[derive(Clone)]
struct Data {
    resources: i32,
    old_nb_buidings: i32,
    nb_buildings: i32,
    landing_areas: Vec<LandingArea>,
    moon_areas: Vec<MoonArea>,
    num_travel_routes: i32,
    routes: HashSet<Routes>,
    pending_routes: HashSet<PendingRoutes>,
    num_pods: i32,
    pods: Vec<Pods>,
}

fn calculate_resource_cost(distance: f64) -> i32 {
    (distance / 0.1).floor() as i32
}

fn distance_cost(a: i32, b: i32, landing_areas: &Vec<LandingArea>, moon_areas: &Vec<MoonArea>) -> i32 {
    let pos1 = position_x_y(a, landing_areas, moon_areas);
    let pos2 = position_x_y(b, landing_areas, moon_areas);

    if pos1 == (0, 0) || pos2 == (0, 0) {
        return 0; // Si l'un des bâtiments n'est pas trouvé
    }

    let distance: f64 = euclidean_distance(pos1, pos2);

    let total_cost: i32 = calculate_resource_cost(distance);

    total_cost
}

fn calculate_resources(itinerary: &Vec<i32>, landing_areas: &Vec<LandingArea>, moon_areas: &Vec<MoonArea>) -> i32 {
    let mut total_cost: i32 = 0;
    for i in 0..itinerary.len() - 1 {
        let route_cost = distance_cost(itinerary[i], itinerary[i + 1], &landing_areas, &moon_areas);
        total_cost += route_cost;
    }
    total_cost
}

// Fonction pour vérifier si un point est sur un segment entre deux autres points
fn point_on_segment(a: (i32, i32), b: (i32, i32), c: (i32, i32)) -> bool {
    let distance = |p1: (i32, i32), p2: (i32, i32)| -> i32 {
        (p2.0 - p1.0).pow(2) + (p2.1 - p1.1).pow(2)
    };
    distance(b, a) + distance(a, c) == distance(b, c) &&
    a.0 >= b.0.min(c.0) && a.0 <= b.0.max(c.0) &&
    a.1 >= b.1.min(c.1) && a.1 <= b.1.max(c.1)
}

// Fonction pour déterminer l'orientation d'un triplet de points
fn orientation(a: (i32, i32), b: (i32, i32), c: (i32, i32)) -> i32 {
    let val = (b.1 - a.1) * (c.0 - b.0) - (b.0 - a.0) * (c.1 - b.1);
    if val == 0 { 0 } else if val > 0 { 1 } else { 2 }
}

// Vérification si deux segments se croisent
fn segments_intersect(a: (i32, i32), b: (i32, i32), c: (i32, i32), d: (i32, i32)) -> bool {
    let o1 = orientation(a, b, c);
    let o2 = orientation(a, b, d);
    let o3 = orientation(c, d, a);
    let o4 = orientation(c, d, b);

    if o1 != o2 && o3 != o4 {
        return true;
    }

    if o1 == 0 && point_on_segment(c, a, b) { return true; }
    if o2 == 0 && point_on_segment(d, a, b) { return true; }
    if o3 == 0 && point_on_segment(a, c, d) { return true; }
    if o4 == 0 && point_on_segment(b, c, d) { return true; }

    false
}

fn check_intersection(
    start_coords: (i32, i32),
    next_coords: (i32, i32),
    existing_routes: &HashSet<Routes>,
    game_data: &Data,
) -> bool {
    for route in existing_routes {
        let existing_start = position_x_y(route.id_building_in, &game_data.landing_areas, &game_data.moon_areas);
        let existing_end = position_x_y(route.id_building_out, &game_data.landing_areas, &game_data.moon_areas);

        if start_coords != existing_start && start_coords != existing_end && next_coords != existing_start && next_coords != existing_end {
            if segments_intersect(start_coords, next_coords, existing_start, existing_end) {
                return true;
            }
        }
    }
    false
}


fn position_x_y(building_id: i32, landing_areas: &Vec<LandingArea>, moon_areas: &Vec<MoonArea>) -> (i32, i32) {
    // Vérifiez d'abord dans les aires d'atterrissage
    for la in landing_areas {
        if building_id == la.get_id() {
            return la.get_position()
        }
    }

    // Si non trouvé, vérifiez dans les zones lunaires
    for ma in moon_areas {
        if building_id == ma.get_id() {
            return ma.get_position()
        }
    }

    (0, 0) // Retourne 0 si l'identifiant de bâtiment n'est trouvé nulle part
}

fn find_alternative_paths(
    graph: &HashMap<i32, Vec<(i32, f64)>>,
    dist: &HashMap<i32, f64>,
    current_node: i32,
    existing_routes: &HashSet<Routes>,
    game_data: &Data,
) -> Vec<i32> {
    let mut alternative_paths = Vec::new();
    
    // Obtenez les voisins du nœud courant
    if let Some(neighbors) = graph.get(&current_node) {
        for &(neighbor, weight) in neighbors {
            // Calculez le coût total du chemin alternatif
            let total_cost = dist.get(&current_node).unwrap_or(&f64::MAX) + weight;

            // Vérifiez s'il y a une intersection avec les itinéraires existants
            let current_coords = position_x_y(current_node, &game_data.landing_areas, &game_data.moon_areas);
            let neighbor_coords = position_x_y(neighbor, &game_data.landing_areas, &game_data.moon_areas);
            if !check_intersection(current_coords, neighbor_coords, existing_routes, &game_data) {
                // Si le chemin n'interfère pas avec les itinéraires existants, nous pouvons l'ajouter
                if total_cost < *dist.get(&neighbor).unwrap_or(&f64::MAX) {
                    alternative_paths.push(neighbor);
                }
            }
        }
    }
    alternative_paths
}

// La distance entre deux bâtiments est calculée pour déterminer le coût du déplacement. Le coût peut être influencé par la distance entre deux bâtiments et le type de connexion.
fn euclidean_distance(pos1: (i32, i32), pos2: (i32, i32)) -> f64 {
    (((pos1.0 - pos2.0).pow(2) + (pos1.1 - pos2.1).pow(2)) as f64).sqrt()
}

fn dijkstra(
    start: i32,
    game_data: &Data,
    destination_type: i32,
    existing_routes: &mut HashSet<Routes>,
) -> Option<Vec<i32>> {
    let mut dist: HashMap<i32, f64> = HashMap::new(); // Distances minimales de la position de départ à chaque point du graphe
    let graph = build_graph(&game_data);
    let mut prev: HashMap<i32, i32> = HashMap::new(); // Pour stocker les prédecesseurs

    // Initialiser les distances à l'infini
    for la in &game_data.landing_areas {
        dist.insert(la.id, f64::MAX);
    }
    for ma in &game_data.moon_areas {
        dist.insert(ma.id, f64::MAX);
    }

    let mut heap = BinaryHeap::new();
    heap.push(State { cost: 0.0, position: start });
    dist.insert(start, 0.0);

    while let Some(State { cost, position }) = heap.pop() {
        if cost > *dist.get(&position).unwrap_or(&f64::MAX) {
            continue; // Si on a trouvé un chemin plus court vers cette position, on passe au nœud suivant
        }

        // Vérifier les connexions depuis la position actuelle
        if let Some(neighbors) = graph.get(&position) {
            for &(next_position, weight) in neighbors {
                let next_cost = cost + weight;

                let start_coords = position_x_y(position, &game_data.landing_areas, &game_data.moon_areas);
                let next_coords = position_x_y(next_position, &game_data.landing_areas, &game_data.moon_areas);

                // Vérifier les intersections avec les itinéraires existants
                if game_data.nb_buildings < 50 {
                    if check_intersection(start_coords, next_coords, &existing_routes, &game_data) {
                        // Itinéraire alternatif
                        let alternative_paths = find_alternative_paths(&graph, &dist, position, &existing_routes, game_data);
                        for alt in alternative_paths {
                            let alt_weight = dist.get(&alt).unwrap_or(&f64::MAX);
                            let alt_cost = next_cost + alt_weight;
    
                            // Vérifier si ce chemin alternatif est meilleur
                            if alt_cost < *dist.get(&alt).unwrap_or(&f64::MAX) {
                                dist.insert(alt, alt_cost);
                                prev.insert(alt, position);
                                heap.push(State { cost: alt_cost, position: alt });
                            }
                        }
                    } else {
                        let route = Routes::new(position, next_position);
                        if !existing_routes.contains(&route) {
                            existing_routes.insert(route);
                        }

                        // Si aucun problème d'intersection, continuer normalement
                        if next_cost < *dist.get(&next_position).unwrap_or(&f64::MAX) {
                            dist.insert(next_position, next_cost);
                            prev.insert(next_position, position);
                            heap.push(State { cost: next_cost, position: next_position });
                        }
                    }
                } else {
                    if next_cost < *dist.get(&next_position).unwrap_or(&f64::MAX) {
                        dist.insert(next_position, next_cost);
                        prev.insert(next_position, position);
                        heap.push(State { cost: next_cost, position: next_position });
                    }
                }
            }
        }
    }

    // Retracer le chemin depuis la destination jusqu'au départ
    let mut path = Vec::new();
    let mut current = destination_type;

    while let Some(&prev_node) = prev.get(&current) {
        path.push(current);
        current = prev_node;
    }

    // Ajouter le point de départ
    path.push(start);
    path.reverse(); // Inverser pour avoir l'ordre du chemin

    // Vérifier si le chemin a bien été trouvé
    if path.len() > 1 {
        Some(path) // Retourner le chemin trouvé
    } else {
        None // Si aucun chemin n'a été trouvé, retourner None
    }
}

#[derive(Clone, Debug, Hash, Eq, PartialEq)]
struct PendingRoutes {
    building_in: i32,
    building_out: i32,
    cost: i32,
}

fn handle_new_buildings(
    game_data: &mut Data, 
    landing_areas: &Vec<LandingArea>, 
    moon_areas: &Vec<MoonArea>, 
    routes: &mut HashSet<Routes>,
    pending_routes: &mut HashSet<PendingRoutes>,
    ret: &mut String,
    my_resources: &mut i32,
) {
    for la in landing_areas {
        for (astronaut_type, _) in &la.astronauts_by_types {
            for ma in moon_areas {
                let type_moon = MoonArea::get_type(ma);
                if type_moon == *astronaut_type {
                    if *my_resources > 5000  && game_data.nb_buildings == 144 {
                        let teleport = Action::Teleport { entrance: la.get_id(), exit: ma.get_id() };
                        ret.push_str(&format!("{}", teleport));
                        break ;
                    }                    
                    if let Some(itinerary) = dijkstra(la.get_id(), game_data, ma.get_id(), routes) {
                        let mut cost_tube = 0;
                        // If there is more than one stop in the itinerary
                        if itinerary.len() > 2 {
                            cost_tube = calculate_resources(&itinerary, &landing_areas, &moon_areas);
                            let mut cost = 0;
                            for i in 0..itinerary.len() - 1 {
                                cost = distance_cost(itinerary[i], itinerary[i + 1], &landing_areas, &moon_areas);
                                // if *my_resources > cost {
                                    let tube_action = Action::Tube {
                                        from: itinerary[i],
                                        to: itinerary[i + 1],
                                    };
                                    ret.push_str(&format!("{}", tube_action));
                                    *my_resources -= cost;
                                } else {
                                    // On garde les routes qu'on a pas pu tracer
                                    pending_routes.insert(PendingRoutes {
                                        building_in: itinerary[i],
                                        building_out: itinerary[i + 1],
                                        cost: cost,
                                    });
                                }
                            }
                        } else {
                            // Direct route from landing area to moon area
                            cost_tube = distance_cost(la.get_id(), ma.get_id(), &landing_areas, &moon_areas);
                            // if *my_resources > cost_tube {
                                let tube_action = Action::Tube {
                                    from: la.get_id(),
                                    to: ma.get_id(),
                                };
                                ret.push_str(&format!("{}", tube_action));
                                *my_resources -= cost_tube;
                            // } else {
                            //     pending_routes.insert(PendingRoutes {
                            //         building_in: la.get_id(),
                            //         building_out: ma.get_id(),
                            //         cost: cost_tube,
                            //     });
                            // }
                        }
                        game_data.num_pods += 1;
                        
                        let pod_action = Action::Pod {
                            pod_id: game_data.num_pods,
                            stops: itinerary.clone(),
                        };
                        ret.push_str(&format!("{}", pod_action));
                    }
                }
            }
        }
    }
}

fn handle_no_new_buildings(
    game_data: &mut Data, 
    pods: &mut Vec<Pods>, 
    landing_areas: &Vec<LandingArea>, 
    moon_areas: &Vec<MoonArea>,
    pending_routes: &mut HashSet<PendingRoutes>, 
    ret: &mut String,
    my_resources: &mut i32,
) {
    let wait_action = Action::Wait;
    
    // Parcourir les routes en attente
    for pod in pods.iter_mut() { // Utilisation de iter_mut() pour modifier les pods
        let itinerary = pod.get_itinerary();

        for i in 0..itinerary.len() - 1 {
            let route_cost = distance_cost(itinerary[i], itinerary[i + 1], &landing_areas, &moon_areas);
            let upgrade_cost = route_cost * 2;

            if *my_resources >= upgrade_cost + 1000 {
                let upgrade_action = Action::Upgrade {
                    from: itinerary[i],
                    to: itinerary[i + 1],
                };
                ret.push_str(&format!("{}", upgrade_action));

                game_data.num_pods += 1;
                let pod_action = Action::Pod {
                    pod_id: game_data.num_pods,
                    stops: itinerary.clone(),
                };
                ret.push_str(&format!("{}", pod_action));
                
                // Mettre à jour le pod dans la liste après l'action
                pod.update_itinerary(itinerary.clone()); 
            } else if *my_resources >= 1000 {
                game_data.num_pods += 1;
                let pod_action = Action::Pod {
                    pod_id: game_data.num_pods,
                    stops: itinerary.clone(),
                };
                ret.push_str(&format!("{}", pod_action));

                // Mettre à jour le pod dans la liste après l'action
                pod.update_itinerary(itinerary.clone());
            } else {
                ret.push_str(&format!("{}", wait_action));
            }
        }
    }

    // Gestion des routes en attente
    pending_routes.retain(|pending_route| {
        if *my_resources >= pending_route.cost {
            let tube_action = Action::Tube {
                from: pending_route.building_in,
                to: pending_route.building_out,
            };
            ret.push_str(&format!("{}", tube_action));
    
            *my_resources -= pending_route.cost;

            false // Ne pas conserver cette route, elle est construite
        } else {
            true // Conserver la route en attente
        }
    });

    ret.push_str(&format!("{}", wait_action));
}


fn start_algorithm(
    game_data: &mut Data, 
    landing_areas: &Vec<LandingArea>, 
    moon_areas: &Vec<MoonArea>, 
    pods: &mut Vec<Pods>, 
    routes: &mut HashSet<Routes>,
    pending_routes: &mut HashSet<PendingRoutes>
) -> String {
    // Gestion de priorite
    let mut ret: String = String::new();
    let mut my_resources: i32 = game_data.resources;

    if game_data.nb_buildings == 3 {
        ret.push_str("TUBE 0 1; TUBE 0 2; POD 0 1 0 2;");
        handle_no_new_buildings(game_data, pods, landing_areas, moon_areas, pending_routes, &mut ret, &mut my_resources);
    }
    else if game_data.old_nb_buidings == game_data.nb_buildings {
    // pas de nouveau building
        handle_no_new_buildings(game_data, pods, landing_areas, moon_areas, pending_routes, &mut ret, &mut my_resources);
    // nouveau buildings
    } else if game_data.old_nb_buidings < game_data.nb_buildings {
        handle_new_buildings(game_data, landing_areas, moon_areas, routes, pending_routes, &mut ret, &mut my_resources);
    }
    ret
}

fn main() {
    let mut travel_routes: HashSet<Routes> = HashSet::new();
    let mut landing_areas: Vec<LandingArea> = Vec::new();
    let mut moon_areas: Vec<MoonArea> = Vec::new();
    let mut pods: Vec<Pods> = Vec::new();
    let mut old_nb_buildings: i32 = 0;
    let mut pending_routes: HashSet<PendingRoutes> = HashSet::new();

    loop {

        let mut input_line = String::new();
        io::stdin().read_line(&mut input_line).unwrap();
        let resources = parse_input!(input_line, i32);
        let mut input_line = String::new();
        io::stdin().read_line(&mut input_line).unwrap();
        let num_travel_routes = parse_input!(input_line, i32);
        
        for _ in 0..num_travel_routes as usize {
            let mut input_line = String::new();
            io::stdin().read_line(&mut input_line).unwrap();
            let inputs = input_line.split_whitespace().collect::<Vec<_>>();
            let building_id_1 = parse_input!(inputs[0], i32);
            let building_id_2 = parse_input!(inputs[1], i32);
            let capacity = parse_input!(inputs[2], i32);

        }

        let mut input_line = String::new();
        io::stdin().read_line(&mut input_line).unwrap();
        let num_pods = parse_input!(input_line, i32);

        
        for _ in 0..num_pods as usize {
            let mut input_line = String::new();
            io::stdin().read_line(&mut input_line).unwrap();
            let pod_properties = input_line.split_whitespace().collect::<Vec<_>>();

            let pod_id = parse_input!(pod_properties[0], i32);
            let pod_stop = parse_input!(pod_properties[1], i32);

            let itinerary: Vec<i32> = pod_properties[2..]
                .iter()
                .map(|s| s.parse::<i32>().unwrap())
                .collect();
            if pods.iter().any(|pod| pod.id == pod_id) {
                    continue; // Si le pod existe, on le saute
            }
            let new_pod = Pods {
                id: pod_id,
                pod_stops: pod_stop,
                capacity: 0,
                itinerary,
            };
            pods.push(new_pod);
        }

        let mut input_line = String::new();
        io::stdin().read_line(&mut input_line).unwrap();
        let num_new_buildings = parse_input!(input_line, i32);
        if num_new_buildings != landing_areas.len() as i32 + moon_areas.len() as i32 {
            old_nb_buildings = landing_areas.len() as i32 + moon_areas.len() as i32
        }
        for _ in 0..num_new_buildings as usize {
            let mut input_line = String::new();
            io::stdin().read_line(&mut input_line).unwrap();
            let building_properties = input_line.split_whitespace().collect::<Vec<_>>();
            let type_building = parse_input!(building_properties[0], i32);
            let id_building = parse_input!(building_properties[1], i32);
            let position: (i32, i32) = (
                parse_input!(building_properties[2], i32), 
                parse_input!(building_properties[3], i32)
            );

            let mut astronauts_by_types: HashMap<i32, i32> = HashMap::new();

            if type_building != 0 {
                let moon_area = MoonArea {
                    id: id_building,
                    type_moon_area: type_building,
                    position,
                };
                moon_areas.push(moon_area);
            } else {

                let nb_astronauts = parse_input!(building_properties[4], i32);

                for astronaut_type_str in &building_properties[5..] {
                    let astronaut_type = astronaut_type_str.parse::<i32>().unwrap();
                    // Incrémente le compteur pour le type d'astronaute
                    *astronauts_by_types.entry(astronaut_type).or_insert(0) += 1;
                }

                let landing_area = LandingArea {
                    id: id_building,
                    position,
                    nb_astronaut: nb_astronauts,
                    astronauts_by_types,
                };
                landing_areas.push(landing_area);
            }
        }
        
        let mut game_data = Data {
            resources,
            old_nb_buidings: old_nb_buildings,
            nb_buildings: landing_areas.len() as i32 + moon_areas.len() as i32,
            landing_areas: landing_areas.clone(),
            moon_areas: moon_areas.clone(),
            num_travel_routes,
            routes: travel_routes.clone(),
            pending_routes: pending_routes.clone(),
            num_pods,
            pods: pods.clone(),
        };


        // print_data(&game_data);
        let ret: String = start_algorithm(&mut game_data, &landing_areas, &moon_areas, &mut pods, &mut travel_routes, &mut pending_routes);
        println!("{}", ret);
    }
}

fn print_data(game_data: &Data) {
    eprintln!(" => Resources: {}", game_data.resources);
    eprintln!(" => Number of buildings before : {}", game_data.old_nb_buidings);
    eprintln!(" => Number of buildings: {}", game_data.nb_buildings);
    eprintln!(" => Number of travel routes: {}", game_data.num_travel_routes);
    eprintln!(" => Number of pods: {}", game_data.num_pods);

    // Afficher piste d'atterissage
    for la in &game_data.landing_areas {
        eprintln!(" => Landing Area - ID: {}, Position: {:?}, Astronauts: {}", la.id, la.position, la.nb_astronaut);
        for (k, v) in &la.astronauts_by_types {
            eprintln!("      => Type {}, Number {}", k, v);
        }
    }

    // Afficher station lunaire
    for ma in &game_data.moon_areas {
        eprintln!(" => Moon Area - ID: {}, Type: {}, Position: {:?}", ma.id, ma.type_moon_area, ma.position);
    }

    // Afficher les pods
    for pod in &game_data.pods {
        eprintln!(" => Pod - ID: {}, Stops: {}, Itinerary: {:?}", pod.id, pod.pod_stops, pod.itinerary);
    }

    // Afficher les routes
    for route in &game_data.routes {
        eprintln!(" => Route - From: {}, To: {}", route.id_building_in, route.id_building_out);
    }

    for pending_route in &game_data.pending_routes {
        eprintln!(" => Pending Route - From: {}, To: {}", pending_route.building_in, pending_route.building_out);
    }
}


fn build_graph(game_data: &Data) -> HashMap<i32, Vec<(i32, f64)>> {
    let mut graph: HashMap<i32, Vec<(i32, f64)>> = HashMap::new();

    // Construire le graphe pour les connexions entre zones d'atterrissage et zones lunaires
    for la in &game_data.landing_areas {
        let (la_x, la_y) = la.get_position();
        let la_id = la.id;

        for ma in &game_data.moon_areas {
            let (ma_x, ma_y) = ma.get_position();
            let ma_id = ma.id;
            let distance = (((ma_x - la_x).pow(2) + (ma_y - la_y).pow(2)) as f64).sqrt();

            // Ajouter la connexion entre la zone d'atterrissage et la zone lunaire si elles ne sont pas déjà connectées
            graph.entry(la_id).or_insert(Vec::new()).push((ma_id, distance));
            graph.entry(ma_id).or_insert(Vec::new()).push((la_id, distance));
        }
    }

    // Ajouter les connexions entre toutes les zones lunaires
    for ma in &game_data.moon_areas {
        let (ma_x, ma_y) = ma.get_position();
        let ma_id = ma.id;

        for other_ma in &game_data.moon_areas {
            if ma_id != other_ma.id {
                let (other_ma_x, other_ma_y) = other_ma.get_position();
                let distance = (((other_ma_x - ma_x).pow(2) + (other_ma_y - ma_y).pow(2)) as f64).sqrt();

                // Ajouter la connexion entre deux zones lunaires
                graph.entry(ma_id).or_insert(Vec::new()).push((other_ma.id, distance));
            }
        }
    }

    // Ajouter les connexions entre toutes les zones d'atterrissage
    for la in &game_data.landing_areas {
        let (la_x, la_y) = la.get_position();
        let la_id = la.id;

        for other_la in &game_data.landing_areas {
            if la_id != other_la.id {
                let (other_la_x, other_la_y) = other_la.get_position();
                let distance = (((other_la_x - la_x).pow(2) + (other_la_y - la_y).pow(2)) as f64).sqrt();

                // Ajouter la connexion entre deux zones d'atterrissage
                graph.entry(la_id).or_insert(Vec::new()).push((other_la.id, distance));
            }
        }
    }
    graph
}


fn print_graph(graph: &HashMap<i32, Vec<(i32, f64)>>) {
    for (node, edges) in graph {
        let edges_str: Vec<String> = edges
            .iter()
            .map(|(neighbor, weight)| format!("{} ({} units)", neighbor, weight))
            .collect();
        eprintln!("ID {}: {:?}", node, edges_str);
    }
}