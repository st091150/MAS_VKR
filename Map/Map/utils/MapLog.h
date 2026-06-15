#pragma once

#include <QLoggingCategory>

// Категории логирования для приложения Map.
// Используем для отделения debug/диагностики от прод-вывода.

Q_DECLARE_LOGGING_CATEGORY(logUi)
Q_DECLARE_LOGGING_CATEGORY(logRoute)
Q_DECLARE_LOGGING_CATEGORY(logPipeline)
