import numpy as np

def cluster_points(
        points,
        min_points=1,
        noise_only=False,
        **kwargs
):
    """
    noise_only:         use geometric clustering for noise filtering only(do not decompose the object into subgroups)
    """
    clusters = [list(np.arange(len(points)))]
    clusters = cluster_trimesh(
        points, clusters, noise_only=noise_only, **kwargs
    )

    clusters = sort_clusters(
        tuple(cluster) for cluster in clusters if len(cluster) >= min_points
    )

    return clusters


def sort_clusters(groups):
    return sorted(groups, key=len, reverse=True)


def geometric_cluster_filter(points, **kwargs):
    for cluster in cluster_points(points, **kwargs):
        yield cluster


def downsample_cluster(cluster, num_pts=1000):
    if len(cluster) <= num_pts:
        return cluster
    else:
        np.random.shuffle(cluster)
        return cluster[:num_pts]


##################################################

DEFAULT_RADIUS = 5e-2  # 3e-2 | 5e-2


def cluster_trimesh(
        points, groups=None, radius=DEFAULT_RADIUS, min_points=10, **kwargs
):
    import trimesh

    if groups is None:
        groups = [list(np.arange(len(points)))]
    new_groups = []
    for group in groups:
        sub_groups_idx = trimesh.grouping.clusters(points, radius)
        nontrivial_sub_groups_idx = [g for g in sub_groups_idx if len(g)>min_points]
        new_groups.extend(
            remove_outliers_noise_only(group, points, nontrivial_sub_groups_idx, **kwargs)
        )
    return sort_clusters(new_groups)


def remove_outlier_group(points, sub_groups_idx, dist_threshold=0.2):
    """
    Prunes clusters in a group with centroids too far away from the largest centroid
    :param points:
    :param sub_groups_idx:
    :param dist_threshold: largest distance allowed between subgroups
    :return:
    """
    if len(sub_groups_idx) <= 1:
        return sub_groups_idx
    # TODO: could instead use the distance between the closest pair of points
    points = np.asarray(points)
    cluster_mean = [
        np.take(points, sub_group_idx, 0).mean(0) for sub_group_idx in sub_groups_idx
    ]
    largest_cluster = np.argmax(
        [len(sub_group_idx) for sub_group_idx in sub_groups_idx]
    )
    sub_groups_idx = [
        sub_group_idx
        for i, sub_group_idx in enumerate(sub_groups_idx)
        if np.linalg.norm(cluster_mean[i] - cluster_mean[largest_cluster])
           <= dist_threshold
    ]
    return sub_groups_idx


def remove_outliers_noise_only(
        group, points, sub_groups_idx, noise_only=True, **kwargs
):
    if noise_only:
        sub_groups_idx = remove_outlier_group(points, sub_groups_idx, **kwargs)
    sub_groups = [
        np.take(group, sub_group_idx).tolist() for sub_group_idx in sub_groups_idx
    ]
    if noise_only:
        sub_groups = [sum(sub_groups, [])]  # merge subgroups into one
    return sub_groups
