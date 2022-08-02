from src.models.patchnetvlad.models_generic import get_backend as get_netvlad_backend
from src.models.patchnetvlad.models_generic import get_model as get_netvlad_model


def create_netvlad_config(config):
    netvlad_config = {}
    netvlad_config['pooling'] = config.name
    netvlad_config['num_clusters'] = config.num_clusters
    netvlad_config['patch_sizes'] = config.patch_sizes
    netvlad_config['strides'] = config.strides
    netvlad_config['num_pcs'] = config.num_pcs

    return netvlad_config


def get_model(config):
    if config.name == 'netvlad':
        encoder_dim, encoder = get_netvlad_backend()
        netvlad_config = create_netvlad_config(config)
        model = get_netvlad_model(encoder, encoder_dim, netvlad_config, append_pca_layer=True)
    else:
        raise NotImplementedError(f"Given model name {config.name} is not implemented!")

    return model
