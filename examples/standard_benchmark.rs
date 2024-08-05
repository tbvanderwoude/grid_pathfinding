use csv::ReaderBuilder;
use grid_pathfinding::PathingGrid;
use grid_util::grid::Grid;
use grid_util::point::Point;
use grid_util::BoolGrid;
use serde::Deserialize;
use std::error::Error;
use std::fs::{self, File};
use std::io::{self, BufRead};
use std::path::Path;
use std::time::{Duration, Instant};
use walkdir::WalkDir;

#[derive(Debug, Deserialize)]
struct Scenario {
    id: u32,
    file_name: String,
    w: u32,
    h: u32,
    x1: u32,
    y1: u32,
    x2: u32,
    y2: u32,
    distance: f64,
}

fn load_benchmark(name: &str) -> (BoolGrid, Vec<(Point, Point)>) {
    let map_str = fs::read_to_string(Path::new(&format!("./maps/{}.map", name)))
        .expect("Could not read scenario file");

    let file = File::open(Path::new(&format!("./scenarios/{}.map.scen", name)))
        .expect("Could not open scenario file");

    // Create a buffer reader to read lines
    let reader = io::BufReader::new(file);
    let mut lines = reader.lines();

    // Skip the first line
    lines.next();

    // Create a CSV reader with tab delimiter from remaining lines
    let remaining_data = lines.collect::<Result<Vec<_>, _>>().unwrap().join("\n");

    let mut csv_reader = ReaderBuilder::new()
        .delimiter(b'\t')
        .has_headers(false)
        // .flexible(true)
        .from_reader(remaining_data.as_bytes());
    // Initialize an empty vector to store the parsed data
    let mut data_array: Vec<(Point, Point)> = Vec::new();

    // Iterate over the records in the file
    for result in csv_reader.deserialize() {
        let record: Scenario = result.expect("Could not parse scenario record");
        let start = Point::new(record.y1 as i32, record.x1 as i32);
        let goal = Point::new(record.y2 as i32, record.x2 as i32);
        data_array.push((start, goal));
    }

    let lines: Vec<&str> = map_str.lines().collect();
    let parse_line = |line: &str| -> usize {
        line.split_once(' ')
            .unwrap()
            .1
            .parse::<usize>()
            .expect("Could not parse value")
    };

    let w = parse_line(&lines[1]);
    let h = parse_line(&lines[2]);

    let offset = 4;
    let mut bool_grid: BoolGrid = BoolGrid::new(w, h, false);
    for y in 0..bool_grid.height() {
        for x in 0..bool_grid.width() {
            // Not sure why x, y have to be swapped here...
            let tile_val = lines[offset + x].as_bytes()[y];
            let val = ![b'.', b'G'].contains(&tile_val);
            bool_grid.set(x, y, val);
        }
    }
    (bool_grid, data_array)
}

fn get_benchmark_names() -> Vec<String> {
    let root = Path::new("maps/");
    let root = root
        .canonicalize()
        .expect("Failed to canonicalize root path");
    let mut names = Vec::new();
    for entry in WalkDir::new(&root).into_iter().skip(2) {
        let path_str = entry.expect("Could not get dir entry");
        let name = path_str
            .path()
            .strip_prefix(&root)
            .unwrap()
            .to_str()
            .unwrap()
            .split_once('.')
            .unwrap()
            .0;
        names.push(name.to_owned());
    }
    names
}

fn main() {
    let benchmark_names = get_benchmark_names();
    let mut total_time = Duration::ZERO;
    for name in benchmark_names {
        println!("Benchmark name: {}", name);

        let (bool_grid, scenarios) = load_benchmark(name.as_str());
        // for (allow_diag, pruning) in [(false, false), (true, false), (true, true)] {
        for (allow_diag, pruning) in [(true, false)] {
            let mut pathing_grid: PathingGrid =
                PathingGrid::new(bool_grid.width, bool_grid.height, true);
            pathing_grid.grid = bool_grid.clone();
            pathing_grid.allow_diagonal_move = allow_diag;
            pathing_grid.improved_pruning = pruning;
            pathing_grid.update_all_neighbours();
            pathing_grid.generate_components();
            let number_of_scenarios = scenarios.len() as u32;
            let before = Instant::now();
            run_scenarios(&pathing_grid, &scenarios);
            let elapsed = before.elapsed();
            println!(
                "\tElapsed time: {:.2?}; per scenario: {:.2?}",
                elapsed,
                elapsed / number_of_scenarios
            );
            total_time += elapsed;
        }
    }
    println!("\tTotal benchmark time: {:.2?}", total_time);
}

pub fn run_scenarios(pathing_grid: &PathingGrid, scenarios: &Vec<(Point, Point)>) {
    for (start, goal) in scenarios {
        let path: Option<Vec<Point>> = pathing_grid.get_waypoints_single_goal(*start, *goal, false);
        assert!(path.is_some());
    }
}
