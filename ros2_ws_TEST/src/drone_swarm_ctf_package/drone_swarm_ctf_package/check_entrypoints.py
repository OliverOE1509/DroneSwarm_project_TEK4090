import pkg_resources
eps = list(pkg_resources.iter_entry_points('console_scripts'))
print('All entry points:')
[print(f'{ep.name}: {ep.module_name}') for ep in eps if 'drone' in ep.name]