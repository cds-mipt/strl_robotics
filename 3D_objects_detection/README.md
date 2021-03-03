# Цель модуля 3D objects detection 
Цель модуля - сегментировать на кадре кнопки лифта и распознать соответствующие надписи, после чего с использованием RGBD сенсора определить пространственные положения объектов в системе координат камеры.

# Основные этапы:
1. Разметка данных через фреймворк [cvat](https://github.com/openvinotoolkit/cvat).
    - видеоматериалы для разметки разделены на tasks в рамках одного проекта для гибкого управления labels
    - применен [трекинг](https://github.com/openvinotoolkit/cvat/blob/develop/cvat/apps/documentation/user_guide.md#track-mode-basics) при разметке данных
    - данные выгружаются как датасеты в формате COCO v1.0 вида annotations/ images для каждой task
    - c помощью скрипта produce_train_val_test.py из размеченных tasks сформированы датасеты для тренировки, валидации и теста 
    (данные находятся на cds-server /home/linok_sa/datasets/lift/single_dataset)

2. Сегментация кнопок моделью [SOLOv2](https://github.com/WXinlong/SOLO).
    - [модуль](https://github.com/WXinlong/SOLO/blob/0c689aec145cb0a7a62f14c83b920b65e64faa1e/mmdet/datasets/pipelines/transforms.py#L722), отвечающий за аугментацию при обучении 
    - примеры работы находятся в папке solov2/examples
    
