"""Headless scientific figures and optional GIF, all explicitly synthetic."""

from pathlib import Path


def render_outputs(scenario, plan, trial, output: Path, animate: bool = False) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Patch, Rectangle
    import numpy as np

    grid = scenario.grid
    ox,oy = grid.origin_xy_m
    extent = (ox, ox+grid.width*grid.resolution_m, oy, oy+grid.height*grid.resolution_m)
    occupied = np.zeros((grid.height,grid.width))
    for r,c in grid.blocked:
        occupied[r,c] = 1
    free = grid.free_cells(scenario.robot.body_radius_m+scenario.robot.planning_margin_m)
    clearance = np.zeros_like(occupied)
    for r in range(grid.height):
        for c in range(grid.width):
            if (r,c) not in free and (r,c) not in grid.blocked:
                clearance[r,c] = 1
    points = np.array([grid.to_world(cell) for cell in plan.cells])
    truth = np.array([[row['true_x_m'],row['true_y_m']] for row in trial.samples])
    estimated = np.array([[row['estimated_x_m'],row['estimated_y_m']] for row in trial.samples])

    def draw_map(ax):
        ax.set_facecolor('#f7f7f2')
        ax.imshow(np.ma.masked_where(clearance==0, clearance), origin='lower', extent=extent,
                  cmap=matplotlib.colors.ListedColormap(['#e5e7eb']), vmin=0,vmax=1,interpolation='nearest')
        ax.imshow(np.ma.masked_where(occupied==0, occupied), origin='lower', extent=extent,
                  cmap=matplotlib.colors.ListedColormap(['#303744']),vmin=0,vmax=1,interpolation='nearest')
        ax.plot(points[:,0],points[:,1],'--',color='#687789',lw=1.6,label='P0 planned route')
        ax.scatter(*points[0],color='#087f5b',s=65,marker='o',label='Start',zorder=6)
        ax.scatter(*points[-1],color='#a61e4d',s=95,marker='*',label='Goal',zorder=6)
        goal = Circle(points[-1],scenario.simulation.true_goal_tolerance_m,facecolor='none',edgecolor='#a61e4d',linestyle=':')
        ax.add_patch(goal)
        ax.set(xlim=extent[:2],ylim=extent[2:],xlabel='x [m]',ylabel='y [m]',aspect='equal')
        ax.grid(alpha=0.15)

    fig,(route_ax,error_ax) = plt.subplots(1,2,figsize=(13,5.5),layout='constrained')
    draw_map(route_ax)
    route_ax.plot(truth[:,0],truth[:,1],color='#1864ab',lw=2,label='Executed ground truth')
    route_ax.plot(estimated[:,0],estimated[:,1],color='#e67700',lw=1.5,label='Estimated position')
    route_ax.add_patch(Circle(truth[-1],scenario.robot.body_radius_m,facecolor='#1864ab',alpha=0.4))
    handles,labels = route_ax.get_legend_handles_labels()
    route_ax.legend(handles+[Patch(facecolor='#303744'),Patch(facecolor='#e5e7eb')],
                    labels+['Hard obstacle','Planning clearance mask'],fontsize=7,loc='upper left')
    error_ax.plot([s['time_s'] for s in trial.samples],[s['position_error_m'] for s in trial.samples],color='#a61e4d',lw=1.5)
    error_ax.set(xlabel='Simulation time [s]',ylabel='Planar position error [m]',title='Estimate versus independent simulated truth')
    error_ax.grid(alpha=0.25)
    m = trial.metrics
    error_ax.text(0.04,0.96,f"Status: {m['status']}\nRMSE: {m['localization_rmse_m']:.3f} m\nPlanned: {m['planned_path_length_m']:.2f} m\nExecuted: {m['executed_path_length_m']:.2f} m\nDuration: {m['mission_time_s']:.1f} s",transform=error_ax.transAxes,va='top',fontsize=10)
    fig.suptitle(f"SYNTHETIC DEVELOPMENT DEMO | P0 | {m['localization_model']} | seed {m['seed']}\nNot lunar observations, SLAM, or validated vehicle dynamics",fontsize=12)
    fig.savefig(output/'overview.png',dpi=160)
    fig.savefig(output/'overview.svg')
    plt.close(fig)

    if animate:
        from PIL import Image
        # Decimate only the preview. Full-rate measurements remain in CSV.
        indices = np.unique(np.linspace(0,len(truth)-1,min(90,len(truth)),dtype=int))
        fig,ax = plt.subplots(figsize=(6.5,5.5),layout='constrained')
        draw_map(ax)
        true_line, = ax.plot([],[],color='#1864ab',lw=2,label='Ground truth')
        est_line, = ax.plot([],[],color='#e67700',lw=1.2,label='Estimate')
        body = Circle(truth[0],scenario.robot.body_radius_m,color='#1864ab',alpha=0.6)
        ax.add_patch(body)
        ax.legend(fontsize=7,loc='upper left')
        frames = []
        for i in indices:
            true_line.set_data(truth[:i+1,0],truth[:i+1,1])
            est_line.set_data(estimated[:i+1,0],estimated[:i+1,1])
            body.center = truth[i]
            ax.set_title(f"SYNTHETIC P0 / {m['localization_model']} / seed {m['seed']}\nt = {trial.samples[i]['time_s']:.1f} s | final status: {m['status']}",fontsize=10)
            fig.canvas.draw()
            frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:,:,:3].copy()).convert('P',palette=Image.Palette.ADAPTIVE))
        # This is a time-compressed preview, not real-time playback.
        frames[0].save(output/'trajectory.gif',save_all=True,append_images=frames[1:],duration=90,loop=0)
        plt.close(fig)
