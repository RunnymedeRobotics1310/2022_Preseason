package y2022;

import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

public abstract class AdventOfCodeSolver {

	protected static Scanner      scanner    = new Scanner(System.in);
	protected static List<String> inputLines = new ArrayList<>();

	protected static boolean      debug = false;

	protected void run() {

		readInput();
		parseInput();
		solve();
	}

	protected void readInput() {

		System.out.println("Paste puzzle input below and enter a blank line at the end...");

		while (true) {

			String inputLine = scanner.nextLine();

			if (inputLine.trim().isBlank()) {
				return;
			}

			inputLines.add(inputLine);
		}
	}

	private void solve() {

		System.out.println("Solve Part 1");

		long start = System.currentTimeMillis();
		solvePart1();

		System.out.println("\nPart 1 Solved in " + (System.currentTimeMillis() - start) + "ms");


		System.out.println("\n\nSolve Part 2");

		start = System.currentTimeMillis();
		solvePart2();

		System.out.println("\nPart 2 Solved in " + (System.currentTimeMillis() - start) + "ms");

	}

	protected abstract void parseInput();
	protected abstract void solvePart1();
	protected abstract void solvePart2();

}
