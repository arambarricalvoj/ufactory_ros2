\NeedsTeXFormat{LaTeX2e}
\ProvidesPackage{InforMeX}[2020/04/06]
\LoadClass[12pt]{article}

% -----------------------------------------
% PAQUETES GENERALES
% -----------------------------------------
\usepackage[utf8]{inputenc}
\usepackage[margin=0.7in, bmargin=1in, a4paper]{geometry}
\usepackage{graphicx}
\usepackage{float}
\usepackage{xcolor}
\usepackage{colortbl}
\usepackage{ifthen}
\usepackage{lipsum}
\usepackage[spanish]{babel}
\usepackage[none]{hyphenat}
\usepackage[backend=biber,style=numeric,sorting=none]{biblatex}
\addbibresource{referencias.bib}
\usepackage{hyperref}
\usepackage{enumitem}
\usepackage{amsmath}
\usepackage{amssymb}
\usepackage{booktabs}
\usepackage{svg}
\usepackage{comment}
\usepackage{multirow}
\usepackage{hhline}
\usepackage{caption}

% -----------------------------------------
% ENTORNO LISTING
% -----------------------------------------
\usepackage{listings}

\newfloat{listing}{htbp}{}
\floatname{listing}{Código}

% MATLAB STYLE
\lstset{
  language=Matlab,
  basicstyle=\ttfamily\scriptsize,
  keywordstyle=\color{blue},
  commentstyle=\color{gray},
  stringstyle=\color{orange},
  showstringspaces=false,
  breaklines=true,
  frame=single,
  captionpos=b,
  numbers=left,
  numberstyle=\tiny\color{gray},
  stepnumber=1,
  numbersep=5pt
}

% C++ STYLE (MISMO FORMATO QUE MATLAB)
\lstdefinestyle{cppstyle}{
  language=C++,
  basicstyle=\ttfamily\scriptsize,
  keywordstyle=\color{blue},
  commentstyle=\color{gray},
  stringstyle=\color{orange},
  showstringspaces=false,
  breaklines=true,
  frame=single,
  captionpos=b,
  numbers=left,
  numberstyle=\tiny\color{gray},
  stepnumber=1,
  numbersep=5pt
}

\renewcommand{\lstlistingname}{Código}
\renewcommand{\lstlistlistingname}{Índice de códigos}

% -----------------------------------------
% TIPOGRAFÍA
% -----------------------------------------
\usepackage{csquotes}
\setlength{\parindent}{0pt}

\definecolor{greyscale}{RGB}{128,128,128}
\definecolor{borgon}{HTML}{990033}
\definecolor{good_black}{RGB}{36, 30, 32}
\definecolor{good_green}{RGB}{109, 191, 81}

\usepackage{fontspec}
\setmainfont[
Path=Core/,
BoldFont=1.ttf,
ItalicFont=2.ttf,
BoldItalicFont=3.ttf,
]{4.ttf}

% -----------------------------------------
% COMANDOS PERSONALIZADOS
% -----------------------------------------
\newcommand*{\datos}[3]{
\ifthenelse{\equal{#1}{color}}{
    \begin{flushleft}
        \vspace*{-0.6in}
        \begin{figure}[H]
        \hspace*{-0.567cm}
            \includegraphics[width=0.6\textwidth]{Images/ucm_uned.png}
        \end{figure}
    \end{flushleft}
}{
    \begin{flushleft}
        \vspace*{-0.6in}
        \begin{figure}[H]
        \hspace*{-0.567cm}
            \includegraphics[width=0.35\textwidth]{Images/UCMnegro.jpg}
        \end{figure}
    \end{flushleft}
}
\begin{flushright}
\vspace*{-2.7cm}
\large
\textbf{\uppercase{ ROBÓTICA INDUSTRIAL#3}}\\\textbf{Trabajo final sobre la materia}
\end{flushright}
}

\newcommand*{\adaptar}[2]{
    \ifthenelse{\equal{#1}{si}}{\large}{}
    \ifthenelse{\equal{#2}{si}}{
        \pagecolor{good_black}
        \color{good_green}
    }{}
}

\newcommand*{\firma}[1]{
    \vfill
    \noindent
    \texttt{#1}
    \vspace{2cm}\\
    \textbf{\today}
}

\newcommand*{\derechosUso}{
\vfill
\begin{center}
\small
Queda prohibido copiar, reproducir, distribuir, publicar, transmitir o explotar cualquier parte de este documento sin autorización previa por escrito del autor o del grupo ViSens.
\end{center}
}

\newcommand*{\portada}[6]{
    \thispagestyle{empty}
    \begin{center}
        \vspace*{2cm}
        \includegraphics[width=0.8\textwidth]{Images/ucm_uned.png}
        \includegraphics[width=0.76\textwidth]{Images/visens.jpg}
        \vspace{2cm}

        {\fontsize{30pt}{36pt}\selectfont \textbf{#6}}
        \vspace{2cm}

        {\fontsize{24pt}{36pt}\selectfont \textbf{#1}}
        \vspace{1cm}

        {\fontsize{20pt}{36pt}\selectfont \textbf{Asignatura:} #2}
        \vspace{1cm}

        {\fontsize{20pt}{36pt}\selectfont \textbf{Curso:} #4}
        \vspace{4cm}

        {\Large \textbf{Alumno:} #3}
        \vspace{1cm}

        {\Large \textbf{#5}}
    \end{center}
    \newpage
}

% -----------------------------------------
% PIE DE PÁGINA CON BANNER + NÚMERO DE PÁGINA
% -----------------------------------------
\usepackage{fancyhdr}
\pagestyle{fancy}

\fancyhf{} % limpiar encabezado y pie

% Sin líneas
\renewcommand{\headrulewidth}{0pt}
\renewcommand{\footrulewidth}{0pt}

% Banner centrado y elevado
\fancyfoot[C]{%
  \vspace*{-6pt}%
  \includegraphics[width=0.45\textwidth]{Images/visens.jpg}
}

% Número de página a la derecha
\fancyfoot[R]{\thepage}

% -----------------------------------------
\endinput
