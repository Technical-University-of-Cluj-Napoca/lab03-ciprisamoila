from utils import *
from grid import Grid
from searching_algorithms import *

labels = ["BFS", "DFS", "DLS", "A*", "UCS", "Dijkstra", "IDDFS", "IDA*", "Start", "Reset"]

def run_algo(index: int, draw, grid: Grid, start: Spot, end: Spot) -> None:
    print(f"Running {labels[index]}")
    match index:
        case 0: bfs(draw, grid, start, end)
        case 1: dfs(draw, grid, start, end)
        case 2: dls(draw, grid, start, end, 50)
        case 3: astar(draw, grid, start, end)
        case 4: ucs(draw, grid, start, end)
        case 5: dijkstra(draw, grid, start, end)
        case 6: iddfs(draw, grid, start, end, 100)
        case 7: idastar(draw, grid, start, end)

selected_algo = 0

if __name__ == "__main__":
    pygame.init()
    # setting up how big will be the display window
    WIN = pygame.display.set_mode((WIDTH, HEIGHT))

    # set a caption for the window
    pygame.display.set_caption("Path Visualizing Algorithm")

    WIN.fill(COLORS['TURQUOISE'])

    ROWS = 40  # number of rows
    COLS = 40  # number of columns
    grid = Grid(WIN, ROWS, COLS, GRID_WIDTH, GRID_HEIGHT)

#chat code
    font = pygame.font.SysFont(None, 36)
    area_height = 700
    area_width = 130
    num_buttons = 10

    top_margin = 20
    left_margin = 20
    spacing = 20  # space between buttons

    # Compute button height automatically
    available_height = area_height - 2 * top_margin - (num_buttons - 1) * spacing
    button_height = available_height / num_buttons
    button_width = area_width - 2 * left_margin

    

    # Create list of button rectangles and their text
    buttons = []
    for i, label in enumerate(labels):
        # Compute vertical position
        y = top_margin + i * (button_height + spacing)
        rect = pygame.Rect(GRID_WIDTH + MARGIN + left_margin, y, button_width, button_height)

        text_surf = font.render(label, True, COLORS['WHITE'])
        text_rect = text_surf.get_rect(center=rect.center)

        buttons.append((rect, text_surf, text_rect, label))

    start = None
    end = None

    # flags for running the main loop
    run = True
    started = False

    while run:
        grid.draw()  # draw the grid and its spots
        for event in pygame.event.get():
            # verify what events happened
            if event.type == pygame.QUIT:
                run = False

            if started:
                # do not allow any other interaction if the algorithm has started
                continue  # ignore other events if algorithm started

            if pygame.mouse.get_pressed()[0]:  # LEFT CLICK
                for i, (rect, text_surf, text_rect, label) in enumerate(buttons):
                    if rect.collidepoint(event.pos):
                        if label == "Reset":
                            print("Clearing the grid...")
                            start = None
                            end = None
                            grid.reset()
                        elif label == "Start":
                            for row in grid.grid:
                                for spot in row:
                                    spot.update_neighbors(grid.grid)

                            started = True

                            run_algo(selected_algo, lambda: grid.draw(), grid, start, end)

                            started = False
                        else:
                            selected_algo = i
                            print(f"Selected {labels[i]}")


                pos = pygame.mouse.get_pos()
                row, col = grid.get_clicked_pos(pos)

                if row >= ROWS or row < 0 or col >= COLS or col < 0:
                    continue  # ignore clicks outside the grid

                spot = grid.grid[row][col]
                if not start and spot != end:
                    start = spot
                    start.make_start()
                elif not end and spot != start:
                    end = spot
                    end.make_end()
                elif spot != end and spot != start:
                    spot.make_barrier()

            elif pygame.mouse.get_pressed()[2]:  # RIGHT CLICK
                pos = pygame.mouse.get_pos()
                row, col = grid.get_clicked_pos(pos)
                spot = grid.grid[row][col]
                spot.reset()

                if spot == start:
                    start = None
                elif spot == end:
                    end = None

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and not started:
                    # run the algorithm
                    for row in grid.grid:
                        for spot in row:
                            spot.update_neighbors(grid.grid)

                    started = True

                    run_algo(selected_algo, lambda: grid.draw(), grid, start, end)
                    # here you can call the algorithms
                    # bfs(lambda: grid.draw(), grid, start, end)
                    # dfs(lambda: grid.draw(), grid, start, end)
                    # astar(lambda: grid.draw(), grid, start, end)
                    # ... and the others?
                    started = False


                if event.key == pygame.K_c:
                    print("Clearing the grid...")
                    start = None
                    end = None
                    grid.reset()

        #drawing the buttons
        for i, (rect, text_surf, text_rect, label) in enumerate(buttons):
            if i == num_buttons - 2:
                color = COLORS['DARK_GREEN'] if rect.collidepoint(pygame.mouse.get_pos()) else COLORS['GREEN']
            else:
                color = COLORS['DARK_BLUE'] if rect.collidepoint(pygame.mouse.get_pos()) else COLORS['BLUE']
            pygame.draw.rect(WIN, color, rect, border_radius=5)
            WIN.blit(text_surf, text_rect)
    pygame.quit()
