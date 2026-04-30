# -*- coding: utf-8 -*-
"""
Main exporter for SDF format
Based on FusionSDF by andreasBihlmaier
"""

import adsk
import adsk.core
import adsk.fusion
from pathlib import Path

from .log import set_log_console, log
from .sdf import SDF


def export(design, save_dir, options=None):
    """
    Export Fusion 360 design to SDF format

    Parameters
    ----------
    design: adsk.fusion.Design - The active design
    save_dir: str - Directory to save the exported files
    options: dict - Export options (optional)

    Returns
    ----------
    tuple: (success: bool, message: str)
    """
    options = options or {}

    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        # Setup logging to Fusion console
        console = ui.palettes.itemById('TextCommands')
        if console and not console.isVisible:
            console.isVisible = True
        set_log_console(console)

        log('\n\n--- FusionRobotExporter (SDF) ---\n')

        if not design:
            return False, 'No active Fusion design'

        sdf_dir_path = Path(save_dir)
        if not sdf_dir_path.exists() or not sdf_dir_path.is_dir():
            return False, f'"{sdf_dir_path}" does not exist or is not a directory'

        # Check for existing files
        if sdf_dir_path.joinpath('model.sdf').exists() or sdf_dir_path.joinpath('meshes').exists():
            # In the full implementation, would ask user for confirmation
            pass

        # Use mesh cache if available
        meshes_cache_path = sdf_dir_path / 'meshes_cache'
        if meshes_cache_path.exists():
            log(f'Using meshes_cache directory "{meshes_cache_path}"\n')
        else:
            meshes_cache_path = None

        # Generate SDF
        sdf = SDF(design, meshes_cache_path)
        sdf.print()
        sdf.save(sdf_dir_path)

        return True, f'Successfully exported SDF to {save_dir}'

    except Exception as e:
        import traceback
        return False, f'Export failed: {str(e)}\n{traceback.format_exc()}'
