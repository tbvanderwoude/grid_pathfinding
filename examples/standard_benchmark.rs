use csv::ReaderBuilder;
use grid_pathfinding::PathingGrid;
use grid_util::grid::Grid;
use grid_util::point::Point;
use serde::Deserialize;
use std::error::Error;
use std::fs::{self, File};
use std::io::{self, BufRead};
use std::path::Path;
use std::time::Instant;

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

fn main() {
    let paths = fs::read_dir("./maps").unwrap();

    for path in paths {
        let filename = path.unwrap().file_name();
        let name = filename.to_str().unwrap();
        println!("Name: {}", name);
        let map_str = fs::read_to_string(Path::new(&format!("./maps/{}", name)))
            .expect("Could not read scenario file");

        let file = File::open(Path::new(&format!("./scenarios/{}.scen", name)))
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
        // for (allow_diag, pruning) in [(false, false), (true, false), (true, true)] {
        for (allow_diag, pruning) in [(true, false)] {
            let mut pathing_grid: PathingGrid = PathingGrid::new(w, h, false);
            pathing_grid.allow_diagonal_move = allow_diag;
            pathing_grid.improved_pruning = pruning;
            for y in 0..pathing_grid.height() {
                for x in 0..pathing_grid.width() {
                    // Not sure why x, y have to be swapped here...
                    let tile_val = lines[offset + x].as_bytes()[y];
                    let val = ![b'.', b'G'].contains(&tile_val);
                    pathing_grid.set(x, y, val);
                }
            }

            pathing_grid.generate_components();
            let number_of_scenarios = data_array.len() as u32;
            let before = Instant::now();
            run_scenarios(&pathing_grid, &data_array);
            let elapsed = before.elapsed();
            println!(
                "\tElapsed time: {:.2?}; per scenario: {:.2?}",
                elapsed,
                elapsed / number_of_scenarios
            );
        }
    }
}

pub fn run_scenarios(pathing_grid: &PathingGrid, scenarios: &Vec<(Point, Point)>) {
    for (start, goal) in scenarios {
        let path: Option<Vec<Point>> = pathing_grid.get_waypoints_single_goal(*start, *goal, false);
        assert!(path.is_some());
    }
}
