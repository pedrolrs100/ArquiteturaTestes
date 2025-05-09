package org.firstinspires.ftc.teamcode.common;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.regex.*;
import fi.iki.elonen.NanoHTTPD;

public class ControlHubServer extends NanoHTTPD {
    private static final String BASE_PATH = "/storage/emulated/0/FTCLogs/";

    public ControlHubServer(int port) throws IOException {
        super(port);
        start(NanoHTTPD.SOCKET_READ_TIMEOUT, false);
        System.out.println("Servidor rodando em http://localhost:" + port);
    }

    @Override
    public Response serve(IHTTPSession session) {
        String uri = session.getUri();

        Response response;
        if (uri.equals("/")) {
            response = newFixedLengthResponse(Response.Status.OK, "text/plain", "API do Control Hub funcionando!");
        } else if (uri.equals("/countFiles")) {
            response = countFiles();
        } else if (uri.equals("/countIDs")) {
            response = countTotalIDs();
        } else if (uri.startsWith("/files/")) {
            response = serveFile(uri.replace("/files/", ""));
        } else {
            response = newFixedLengthResponse(Response.Status.NOT_FOUND, "text/plain", "Rota não encontrada!");
        }

        // 🔹 Adiciona suporte a CORS em todas as respostas
        response.addHeader("Access-Control-Allow-Origin", "*"); // Permite requisições de qualquer origem
        response.addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS"); // Métodos permitidos
        response.addHeader("Access-Control-Allow-Headers", "Content-Type, Authorization"); // Headers permitidos

        return response;
    }

    private Response countFiles() {
        File folder = new File(BASE_PATH);
        if (!folder.exists() || !folder.isDirectory()) {
            return newFixedLengthResponse(Response.Status.NOT_FOUND, "text/plain", "Pasta não encontrada!");
        }

        File[] files = folder.listFiles((dir, name) -> name.endsWith(".json"));
        int fileCount = (files != null) ? files.length : 0;

        return createJsonResponse("{ \"totalArquivos\": " + fileCount + " }");
    }

    private Response countTotalIDs() {
        File folder = new File(BASE_PATH);
        if (!folder.exists() || !folder.isDirectory()) {
            return newFixedLengthResponse(Response.Status.NOT_FOUND, "text/plain", "Pasta não encontrada!");
        }

        File[] files = folder.listFiles((dir, name) -> name.endsWith(".json"));
        if (files == null || files.length == 0) {
            return createJsonResponse("{ \"totalIDs\": 0 }");
        }

        int totalIDs = 0;
        for (File file : files) {
            try {
                totalIDs += countIDsInFile(file);
            } catch (Exception e) {
                System.err.println("Erro ao contar IDs no arquivo: " + file.getName() + " - " + e.getMessage());
            }
        }

        return createJsonResponse("{ \"totalIDs\": " + totalIDs + " }");
    }

    private int countIDsInFile(File file) throws IOException {
        int count = 0;


        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String line;
            Pattern pattern = Pattern.compile("\"ID\"\\s*:\\s*(\\d+)");

            while ((line = reader.readLine()) != null) {
                Matcher matcher = pattern.matcher(line);
                while (matcher.find()) {
                    count++;
                }
            }
        }

        return count;
    }

    private Response serveFile(String filePath) {
        try {
            File file = new File(BASE_PATH + filePath);
            if (!file.exists() || file.isDirectory()) {
                return newFixedLengthResponse(Response.Status.NOT_FOUND, "text/plain", "Arquivo não encontrado!");
            }

            FileInputStream fis = new FileInputStream(file);
            return newChunkedResponse(Response.Status.OK, "application/json", fis);
        } catch (Exception e) {
            return newFixedLengthResponse(Response.Status.INTERNAL_ERROR, "text/plain", "Erro ao ler arquivo: " + e.getMessage());
        }
    }

    private Response createJsonResponse(String json) {
        Response response = newFixedLengthResponse(Response.Status.OK, "application/json", json);
        response.addHeader("Access-Control-Allow-Origin", "*");
        response.addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        response.addHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
        return response;
    }
}
